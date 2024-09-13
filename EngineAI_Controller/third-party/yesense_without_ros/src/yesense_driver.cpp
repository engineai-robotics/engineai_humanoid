#include "yesense_driver.h"
#include <map>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/case_conv.hpp>
#include <chrono>

// #define DATA_LOG_ENABLE

namespace yesense
{

    YesenseDriver::YesenseDriver() : port_("/dev/ttyACM0"),
                                     baudrate_(460800),
                                     buffer_size_(4096),
                                     wait_response_flag_(false),
                                     check_respose_flag_(false),
                                     error_respose_cnt_(0),
                                     configured_(false)
    {
        // 数据缓冲区
        data_buffer_ptr_ = boost::shared_ptr<boost::circular_buffer<char>>(new boost::circular_buffer<char>(buffer_size_));

        // 读取串口数据所需的变量
        bytes_ = 0;

        response.response_need = false;
        response.response_found = false;
        response.response_recv_done = false;
        response.len = 0;
        memset((uint8_t *)&yesense_out, 0, sizeof(yesense_out));

        initSerial();

        deseralize_thread_ = boost::thread(boost::bind(&YesenseDriver::_spin, this));
#ifdef DATA_LOG_ENABLE
        data_log_file.open("../data/datalog.csv", std::ios::trunc);
#endif
    }

    YesenseDriver::~YesenseDriver()
    {
        std::cout << "Close yesense device." << std::endl;
        if (serial_.isOpen())
        {
            serial_.close();
        }
        data_buffer_ptr_.reset();

        configured_ = false;
        deseralize_thread_.join();
    }

    void YesenseDriver::run()
    {
        try
        {
            // read data from serial
            if (serial_.available())
            {
                data_ = serial_.read(serial_.available());

                {
                    boost::mutex::scoped_lock lock(m_mutex_);

                    for (int i = 0; i < data_.length(); i++)
                    {
                        data_buffer_ptr_->push_back(data_[i]);
                    }
                }
            }

            usleep(50);
        }
        catch (std::exception &err)
        {
            std::cout << "error in 'run' function in yeseense IMU driver, msg: " << err.what() << std::endl;
        }
    }

    void YesenseDriver::initSerial()
    {
        while (serial_.isOpen() == false)
        {
            try
            {
                std::cout << "port: " << port_.c_str() << "rate: " << baudrate_ << std::endl;
                serial_.setPort(port_);
                serial_.setBaudrate(baudrate_);
                serial::Timeout to = serial::Timeout::simpleTimeout(1000);
                serial_.setTimeout(to);
                serial_.open();
            }
            catch (serial::IOException &e)
            {
                std::cout << "Unable to open serial port: " << serial_.getPort().c_str() << ",  Trying again in 5 seconds." << std::endl;
                usleep(5000000);
            }
        }

        if (serial_.isOpen())
        {
            std::cout << "Serial port: " << serial_.getPort().c_str() << ", initialized and opened." << std::endl;

            configured_ = true;
        }
    }

    void YesenseDriver::_spin()
    {
        try
        {
            this->spin();
        }
        catch (std::exception &err)
        {

            std::cout << "error in 'spin' function in yesense IMU driver, msg: " << err.what() << std::endl;
        }
    }

    void YesenseDriver::spin()
    {

#define RECV_TIMEOUT_THR 20 /*unit is equal to the above rate*/

        uint16_t tid = 0x00;
        uint16_t prev_tid = 0x00;

        uint32_t gps_header_sum;
        uint32_t recv_len = 0;
        uint32_t timeout_cnt = 0;

        while (configured_)
        {
            recv_len = data_buffer_ptr_->size();
            // ROS_INFO("obtain len %d", recv_len);
            if (recv_len > 0)
            {
                // ROS_INFO("recv data len %d,data is:\n", recv_len);
                boost::mutex::scoped_lock lock(m_mutex_);
                for (unsigned int i = 0; i < recv_len; i++)
                {
                    // ROS_INFO("%02x ", (uint8_t)(data_buffer_ptr_->begin())[i]);
                    message_in_[bytes_ + i] = data_buffer_ptr_->begin()[i];
                }
                bytes_ += recv_len;
                data_buffer_ptr_->erase_begin(recv_len);
            }

            /*analysis data from message_in_ when total recieved data length is bigger than PROTOCOL_MIN_LEN*/
            if (bytes_ >= PROTOCOL_MIN_LEN)
            {
                int cnt = bytes_;
                int pos = 0;
                while (cnt > 0)
                {
                    int ret = analysis_data(message_in_ + pos, cnt, &yesense_out, &response);

                    /*未查找到帧头*/
                    if (analysis_done == ret)
                    {
                        pos++;
                        cnt--;
                    }
                    else if (data_len_err == ret)
                    {
                        if (timeout_cnt >= RECV_TIMEOUT_THR)
                        {
                            timeout_cnt = 0;
                            cnt = 0;
                        }

                        break;
                    }
                    else if (crc_err == ret || analysis_ok == ret)
                    {
                        /*删除已解析完的完整一帧*/
                        output_data_header_t *header = (output_data_header_t *)(message_in_ + pos);
                        unsigned int frame_len = header->len + PROTOCOL_MIN_LEN;
                        if (response.response_recv_done)
                        {
                            frame_len = response.len + PROTOCOL_MIN_LEN;
                        }

                        if (analysis_ok == ret)
                        {
                            if (response.response_recv_done)
                            {
                                // ROS_INFO("response recv");
                                boost::mutex::scoped_lock lock(m_response_mutex_);
                                response.response_need = false;
                                response.response_found = false;
                                response.response_recv_done = false;
                            }
                            else
                            {
                                // ROS_INFO("tid is %d", yesense_out.tid);
                                if (prev_tid != 0 && tid > prev_tid && prev_tid != tid - 1)
                                {
                                    std::cout << "Frame losed: prev_TID: " << prev_tid << ", cur_TID: " << tid << std::endl;
                                }

                                prev_tid = tid;
                                boost::mutex::scoped_lock lock(m_mutex_);
#ifdef DATA_LOG_ENABLE
                                auto finish_time = std::chrono::high_resolution_clock::now();

                                auto elapsed_time = std::chrono::duration_cast<std::chrono::nanoseconds>(finish_time - start_time).count() / 1e6;

                                start_time = finish_time;

                                if (data_log_file.is_open())
                                {
                                    if (first_run)
                                    {
                                        first_run = false;
                                        data_log_file << "time,roll,pitch,yaw" << "\n";
                                        data_log_file << elapsed_time << "," << yesense_out.roll << "," << yesense_out.pitch << "," << yesense_out.yaw << "\n";
                                    }
                                    else
                                        data_log_file << elapsed_time << "," << yesense_out.roll << "," << yesense_out.pitch << "," << yesense_out.yaw << "\n";
                                }
                                else
                                    std::cerr << "Unable to open file\n";
#endif
                                acc[0] = yesense_out.accel_x;
                                acc[1] = yesense_out.accel_y;
                                acc[2] = yesense_out.accel_z;

                                gyro[0] = yesense_out.angle_x;
                                gyro[1] = yesense_out.angle_y;
                                gyro[2] = yesense_out.angle_z;

                                quat[0] = yesense_out.quaternion_data0;
                                quat[1] = yesense_out.quaternion_data1;
                                quat[2] = yesense_out.quaternion_data2;
                                quat[3] = yesense_out.quaternion_data3;

                                rpy[0] = yesense_out.roll;
                                rpy[1] = yesense_out.pitch;
                                rpy[2] = yesense_out.yaw;

                                // std::cout << yesense_out.roll << " || " << yesense_out.pitch << " || " << yesense_out.yaw << std::endl;
                            }
                        }

                        cnt -= frame_len;
                        pos += frame_len;
                    }
                }

                // ROS_INFO("analysis once"); if(bytes_ > cnt && cnt > 0)
                memcpy(message_in_, message_in_ + pos, cnt);
                bytes_ = cnt;
            }

            usleep(50);
        }
    }

}
