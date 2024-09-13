#include <serial/serial.h>
#include <boost/circular_buffer.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "analysis_data.h"
#include <iostream>

#include <chrono>
#include <fstream> // Include the file stream library

#include "../../../common/include/cTypes.h"
// #include "cppTypes.h"
#include "../../../common/include/cppTypes.h"

#define ENBALE_DEBUG_OUTPUT 1
#define ENABLE_SERIAL_INPUT 1

#define DATA_BUF_SIZE 1024

namespace yesense
{

    // IMU Data protocal
    /*--------------------------------------------------------------------------------------------------------------
     * 输出协议为：header1(0x59) + header2(0x53) + tid(2B) + payload_len(1B) + payload_data(Nbytes) + ck1(1B) + ck2(1B)
     * crc校验从TID开始到payload data的最后一个字节
     */
    const uint8_t MODE_HEADER1 = 0;
    const uint8_t MODE_HEADER2 = 1;
    const uint8_t MODE_TID_L = 2;
    const uint8_t MODE_TID_H = 3;
    const uint8_t MODE_LENGTH = 4;
    const uint8_t MODE_MESSAGE = 5;
    const uint8_t MODE_CHECKSUM_L = 6; // checksum for msg and topic id
    const uint8_t MODE_CHECKSUM_H = 7; // checksum for msg and topic id

    // gps raw data mode
    const uint8_t MODE_GPS_RAW = 10;

    // IMU Param protocal
    /*--------------------------------------------------------------------------------------------------------------
     * 输入协议为：header1(0x59) + header2(0x53) + class(1B) + id(3-bit) + len(13-bit) + message(Nbytes) + ck1(1B) + ck2(1B)
     * crc校验从class开始到message的最后一个字节
     */
    const uint8_t PORDUCTION_INFO = 0x00;
    const uint8_t BAUDRATE = 0x02;
    const uint8_t OUTPUT_FREEQUENCY = 0x03;
    const uint8_t OUTPUT_CONTENT = 0x04;
    const uint8_t STANDARD_PARAM = 0x05;
    const uint8_t MODE_SETTING = 0x4D;

    typedef enum _Id
    {
        QUERY_STATUS = 0x00,
        CONFIG_MEMERY = 0x01,
        CONFIG_FLASH = 0x02
    } ID;

#pragma pack(1)
    typedef struct
    {
        uint8_t type;
        uint8_t len;
    } yis_tlv_header_t;
#pragma pack()

    class YesenseDriver
    {
    public:
        YesenseDriver();
        ~YesenseDriver();

        void run();
        void setParam();
        bool getConfiguration() { return configured_; };

        Vec3<float> gyro;
        Vec3<float> acc;
        Vec4<float> quat;
        Vec3<float> rpy;

    protected:
        void initSerial();
        void _spin();
        void spin();

    private:
        std::string port_;      // 串口端口
        int baudrate_;          // 波特率
        serial::Serial serial_; // 串口实例

        double time_offset_in_seconds_;
        bool broadcast_tf_;
        double linear_acceleration_stddev_;
        double angular_velocity_stddev_;
        double orientation_stddev_;

        // RingBuffer环形存储区，用于存储从串口中读出的数据
        std::string data_;
        boost::mutex m_mutex_;
        const int buffer_size_;
        boost::shared_ptr<boost::circular_buffer<char>> data_buffer_ptr_;

        /**/
        protocol_info_t yesense_out;

        // State machine variables for spinOnce
        int bytes_;

        bool configured_;

        /* used for syncing the time */
        uint32_t last_sync_time;
        uint32_t last_sync_receive_time;
        uint32_t last_msg_timeout_time;

        uint8_t message_in_[DATA_BUF_SIZE];
        uint8_t gps_buf[DATA_BUF_SIZE];
        uint32_t gps_buf_index;
        std::map<uint32_t, std::string> gps_raw;

        // 查询参数返回值相关
        boost::mutex m_response_mutex_;
        bool wait_response_flag_;
        bool check_respose_flag_;
        int error_respose_cnt_;

        uint8_t param_class_;
        uint8_t param_id_;
        yis_cmd_response_t response;

        std::string param_prev_topic_id_;
        std::string param_prev_topic_cmd_;

        // 数据处理线程
        boost::thread deseralize_thread_;

        // date-test
        std::ofstream data_log_file;
        bool first_run = true;
        std::chrono::system_clock::time_point start_time = std::chrono::high_resolution_clock::now();
    };

}
