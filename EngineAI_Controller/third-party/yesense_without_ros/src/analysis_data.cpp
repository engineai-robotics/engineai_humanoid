#include <stdint.h>
#include <stddef.h>
#include "analysis_data.h"
#include "cstring"

/*------------------------------------------------MARCOS define------------------------------------------------*/
#define PROTOCOL_FIRST_BYTE_POS 0
#define PROTOCOL_SECOND_BYTE_POS 1
#define PROTOCOL_DATA_LEN_POS 4

#define PROTOCOL_TID_LEN 2

#define CRC_CALC_START_POS 2
#define CRC_CALC_LEN(payload_len) ((payload_len) + 3) /*3 = tid(2B) + len(1B)*/
#define PROTOCOL_CRC_DATA_POS(payload_len) (CRC_CALC_START_POS + CRC_CALC_LEN(payload_len))

#define PAYLOAD_POS 5

#define SINGLE_DATA_BYTES 4

#define MAX_DATA_LEN_NUM (unsigned short)255

/*data id define*/
#define IMU_TEMP_ID (unsigned char)0x01
#define ACCEL_ID (unsigned char)0x10
#define ANGLE_ID (unsigned char)0x20
#define MAGNETIC_ID (unsigned char)0x30		/*归一化值*/
#define RAW_MAGNETIC_ID (unsigned char)0x31 /*原始值*/
#define EULER_ID (unsigned char)0x40
#define QUATERNION_ID (unsigned char)0x41
#define UTC_ID (unsigned char)0x50
#define SAMPLE_TIMESTAMP_ID (unsigned char)0x51
#define DATA_READY_TIMESTAMP_ID (unsigned char)0x52
#define LOCATION_ID (unsigned char)0x68 /*high precision location*/
#define SPEED_ID (unsigned char)0x70
#define STATUS_ID (unsigned char)0x80 /*imu status*/

#define GNSS_MASTER_ID (unsigned char)0xc0
#define GNSS_SLAVE_ID (unsigned char)0xf0

/*length for specific data id*/
#define IMU_TEMP_DATA_LEN (unsigned char)2
#define ACCEL_DATA_LEN (unsigned char)12
#define ANGLE_DATA_LEN (unsigned char)12
#define MAGNETIC_DATA_LEN (unsigned char)12
#define MAGNETIC_RAW_DATA_LEN (unsigned char)12
#define EULER_DATA_LEN (unsigned char)12
#define QUATERNION_DATA_LEN (unsigned char)16
#define UTC_DATA_LEN (unsigned char)11
#define SAMPLE_TIMESTAMP_DATA_LEN (unsigned char)4
#define DATA_READY_TIMESTAMP_DATA_LEN (unsigned char)4
#define LOCATION_DATA_LEN (unsigned char)20 /*high precision location data len*/
#define SPEED_DATA_LEN (unsigned char)12
#define STATUS_DATA_LEN (unsigned char)1 /*imu status*/

#define GNSS_MASTER_DATA_LEN (unsigned char)45
#define GNSS_SLAVE_DATA_LEN (unsigned char)6

/*factor for sensor data*/
#define NOT_MAG_DATA_FACTOR 0.000001f
#define MAG_RAW_DATA_FACTOR 0.001f

#define IMU_TEMP_FACTOR 0.01f

/*factor for gnss data*/
#define LONG_LAT_DATA_FACTOR 0.0000001
#define HIGH_PRECI_LONG_LAT_DATA_FACTOR 0.0000000001
#define ALT_DATA_FACTOR 0.001f
#define SPEED_DATA_FACTOR 0.001f

/*------------------------------------------------Variables define------------------------------------------------*/

/*------------------------------------------------Functions declare------------------------------------------------*/
int calc_checksum(unsigned char *data, unsigned short len, unsigned short *checksum);
static int16_t get_int16(uint8_t *buf, uint16_t offset);
static int32_t get_int32(uint8_t *buf, uint16_t offset);
static int64_t get_int64(uint8_t *buf, uint16_t offset);

/*-------------------------------------------------------------------------------------------------------------*/
int parse_data_by_id(payload_data_t header, unsigned char *data, protocol_info_t *info)
{
	int ret = analysis_ok;

	if (NULL == data || (unsigned char)0 == header.data_len || NULL == info)
	{
		return para_err;
	}

	switch (header.data_id)
	{
	case IMU_TEMP_ID:
	{
		if (IMU_TEMP_DATA_LEN == header.data_len)
		{
			uint16_t temp = (((uint16_t)data[1]) << 8) | ((uint16_t)data[0]);
			info->imu_temp = ((float)temp) * IMU_TEMP_FACTOR;
		}
		else
		{
			ret = data_len_err;
		}
	}
	break;

	case ACCEL_ID:
	{
		if (ACCEL_DATA_LEN == header.data_len)
		{
			info->accel_x = get_int32(data, 0) * NOT_MAG_DATA_FACTOR;
			info->accel_y = get_int32(data, SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
			info->accel_z = get_int32(data, SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
		}
		else
		{
			ret = data_len_err;
		}
	}
	break;

	case ANGLE_ID:
	{
		if (ANGLE_DATA_LEN == header.data_len)
		{
			info->angle_x = get_int32(data, 0) * NOT_MAG_DATA_FACTOR;
			info->angle_y = get_int32(data, SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
			info->angle_z = get_int32(data, SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
		}
		else
		{
			ret = data_len_err;
		}
	}
	break;

	case MAGNETIC_ID:
	{
		if (MAGNETIC_DATA_LEN == header.data_len)
		{
			info->mag_x = get_int32(data, 0) * NOT_MAG_DATA_FACTOR;
			info->mag_y = get_int32(data, SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
			info->mag_z = get_int32(data, SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
		}
		else
		{
			ret = data_len_err;
		}
	}
	break;

	case RAW_MAGNETIC_ID:
	{
		if (MAGNETIC_RAW_DATA_LEN == header.data_len)
		{
			info->raw_mag_x = get_int32(data, 0) * MAG_RAW_DATA_FACTOR;
			info->raw_mag_y = get_int32(data, SINGLE_DATA_BYTES) * MAG_RAW_DATA_FACTOR;
			info->raw_mag_z = get_int32(data, SINGLE_DATA_BYTES * 2) * MAG_RAW_DATA_FACTOR;
		}
		else
		{
			ret = data_len_err;
		}
	}
	break;

	case EULER_ID:
	{
		if (EULER_DATA_LEN == header.data_len)
		{
			info->pitch = get_int32(data, 0) * NOT_MAG_DATA_FACTOR;
			info->roll = get_int32(data, SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
			info->yaw = get_int32(data, SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
		}
		else
		{
			ret = data_len_err;
		}
	}
	break;

	case QUATERNION_ID:
	{
		if (QUATERNION_DATA_LEN == header.data_len)
		{
			info->quaternion_data0 = get_int32(data, 0) * NOT_MAG_DATA_FACTOR;
			info->quaternion_data1 = get_int32(data, SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
			info->quaternion_data2 = get_int32(data, SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
			info->quaternion_data3 = get_int32(data, SINGLE_DATA_BYTES * 3) * NOT_MAG_DATA_FACTOR;
		}
		else
		{
			ret = data_len_err;
		}
	}
	break;

	case LOCATION_ID:
	{
		if (LOCATION_DATA_LEN == header.data_len)
		{
			info->gnss.location.latitude = get_int64(data, 0) * HIGH_PRECI_LONG_LAT_DATA_FACTOR;
			info->gnss.location.longtidue = get_int64(data, 8) * HIGH_PRECI_LONG_LAT_DATA_FACTOR;
			info->gnss.location.altidue = get_int32(data, 16) * ALT_DATA_FACTOR;
			// ROS_INFO("location: %.3f, %.3f, %.3f", g_output_info.longtidue, g_output_info.latitude, g_output_info.altidue);
		}
		else
		{
			ret = data_len_err;
		}
	}
	break;

	case UTC_ID:
	{
		if (UTC_DATA_LEN == header.data_len)
		{
			memcpy((uint8_t *)&info->gnss.utc_time.ms, data, UTC_DATA_LEN);
		}
		else
		{
			ret = data_len_err;
		}
	}
	break;

	case SPEED_ID:
	{
		if (SPEED_DATA_LEN == header.data_len)
		{
			info->gnss.vel.vel_e = get_int32(data, 0) * SPEED_DATA_FACTOR;
			info->gnss.vel.vel_n = get_int32(data, 4) * SPEED_DATA_FACTOR;
			info->gnss.vel.vel_u = get_int32(data, 8) * SPEED_DATA_FACTOR;
		}
		else
		{
			ret = data_len_err;
		}
	}
	break;

	case STATUS_ID:
	{
		if (STATUS_DATA_LEN == header.data_len)
		{
			info->fusion_status = *data;
		}
		else
		{
			ret = data_len_err;
		}
	}
	break;

	case GNSS_MASTER_ID:
	{
		ret = GNSS_MASTER_DATA_LEN == header.data_len;

		if (ret)
		{
			uint8_t *buf = data;

			memcpy(&info->gnss.utc_time, buf, 9);
			buf += 9;

			info->gnss.location.latitude = get_int64(buf, 0) * HIGH_PRECI_LONG_LAT_DATA_FACTOR;
			info->gnss.location.longtidue = get_int64(buf, 8) * HIGH_PRECI_LONG_LAT_DATA_FACTOR;
			info->gnss.location.altidue = get_int32(buf, 16) * ALT_DATA_FACTOR;
			buf += 20;

			info->gnss.location_error.lat = get_int16(buf, 0) * 0.001f;
			info->gnss.location_error.lon = get_int16(buf, 2) * 0.001f;
			info->gnss.location_error.alt = get_int16(buf, 4) * 0.001f;
			buf += 6;

			info->gnss.ground_speed = get_int16(buf, 0) * 0.01f;
			info->gnss.yaw = get_int16(buf, 2) * 0.01f;
			buf += 4;

			info->gnss.status = buf[0];
			info->gnss.star_cnt = buf[1];
			buf += 2;

			info->gnss.p_dop = get_int16(buf, 0) * 0.001f;
			info->gnss.site_id = get_int16(buf, 2);
		}
	}
	break;

	case GNSS_SLAVE_ID:
	{
		ret = GNSS_SLAVE_DATA_LEN == header.data_len;

		if (ret)
		{
			uint8_t *buf = data;
			info->gnss_slave.dual_ant_yaw = get_int16(buf, 0) * 0.01f;
			info->gnss_slave.dual_ant_yaw_error = get_int16(buf, 2) * 0.001f;
			info->gnss_slave.dual_ant_baseline_len = get_int16(buf, 4) * 0.001f;
		}
	}
	break;

	case SAMPLE_TIMESTAMP_ID:
	{
		if (SAMPLE_TIMESTAMP_DATA_LEN == header.data_len)
		{
			info->sample_timestamp = *((unsigned int *)data);
		}
		else
		{
			ret = data_len_err;
		}
	}
	break;

	case DATA_READY_TIMESTAMP_ID:
	{
		if (DATA_READY_TIMESTAMP_DATA_LEN == header.data_len)
		{
			info->out_sync_timestamp = *((unsigned int *)data);
		}
		else
		{
			ret = data_len_err;
		}
	}
	break;

	default:
	{
		ret = data_len_err;
	}
	break;
	}

	return ret;
}

/*--------------------------------------------------------------------------------------------------------------
* 输出协议为：header1(0x59) + header2(0x53) + tid(2B) + payload_len(1B) + payload_data(Nbytes) + ck1(1B) + ck2(1B)
* crc校验从TID开始到payload data的最后一个字节
---------------------------------------------------------------------------------------------------------------*/
int analysis_data(unsigned char *data, short len, protocol_info_t *info, yis_cmd_response_t *cmd)
{
	unsigned short payload_len = 0;
	unsigned short check_sum = 0;
	unsigned short pos = 0;
	int ret = analysis_done;

	output_data_header_t *header = NULL;
	payload_data_t *payload = NULL;

	if (NULL == data || 0 >= len)
	{
		return para_err;
	}

	if (len < PROTOCOL_MIN_LEN)
	{
		return data_len_err;
	}

	/*judge protocol header*/
	if (PROTOCOL_FIRST_BYTE == data[PROTOCOL_FIRST_BYTE_POS] &&
		PROTOCOL_SECOND_BYTE == data[PROTOCOL_SECOND_BYTE_POS])
	{
		/*analysis response data first*/
		if (cmd->response_need)
		{
			/**/
			yis_cmd_response_t *response = (yis_cmd_response_t *)(data + PROTOCOL_SECOND_BYTE_POS + 1);
			if (cmd->type == response->type && cmd->op_id == response->op_id)
			{
				cmd->response_found = 1;

				/*check data length*/
				payload_len = response->len;
				if (payload_len + PROTOCOL_MIN_LEN > len)
				{
					return data_len_err;
				}

				/*check crc*/
				calc_checksum(data + CRC_CALC_START_POS, CRC_CALC_LEN(payload_len), &check_sum);
				if (check_sum != *((unsigned short *)(data + PROTOCOL_CRC_DATA_POS(payload_len))))
				{
					return crc_err;
				}

				// for(int i = 0; i < payload_len + PROTOCOL_MIN_LEN; i++)
				// {
				// 	ROS_INFO("%02x", data[i]);
				// }

				// ROS_INFO("analysis done, type %02x, op_id %d, response len %d", response->type, response->op_id, response->len);
				cmd->response_recv_done = true;
				cmd->len = response->len;

				return analysis_ok;
			}
		}

		/*further check*/
		header = (output_data_header_t *)data;
		payload_len = header->len;

		if (payload_len > MAX_DATA_LEN_NUM)
		{
			return analysis_done;
		}

		if (payload_len + PROTOCOL_MIN_LEN > len)
		{
			return data_len_err;
		}

		/*checksum*/
		calc_checksum(data + CRC_CALC_START_POS, CRC_CALC_LEN(payload_len), &check_sum);
		if (check_sum != *((unsigned short *)(data + PROTOCOL_CRC_DATA_POS(payload_len))))
		{
			return crc_err;
		}

		info->tid = header->tid;

		/*analysis payload data*/
		pos = PAYLOAD_POS;
		// ROS_INFO("paylaod len %04x, header len %02x", payload_len, header->len);
		while (payload_len > (unsigned short)0 && pos < (header->len + PAYLOAD_POS))
		{
			payload = (payload_data_t *)(data + pos);
			ret = parse_data_by_id(*payload, (unsigned char *)payload + sizeof(payload_data_t), info);
			if (analysis_ok == ret)
			{
				pos += payload->data_len + sizeof(payload_data_t);
				payload_len -= payload->data_len + sizeof(payload_data_t);
			}
			else
			{
				pos++;
				payload_len--;
			}
		}

		return analysis_ok;
	}
	else
	{
		return analysis_done;
	}
}

/*---------------------------------------------------------------------------------------------------------------*/
int calc_checksum(unsigned char *data, unsigned short len, unsigned short *checksum)
{
	unsigned char check_a = 0; /*数据类型为char，在不为char的时候，某些情况下crc计算时错误的*/
	unsigned char check_b = 0;
	unsigned short i;

	if (NULL == data || 0 == len || NULL == checksum)
	{
		return para_err;
	}

	for (i = 0; i < len; i++)
	{
		check_a += data[i];
		check_b += check_a;
	}

	*checksum = ((unsigned short)(check_b << 8) | check_a);

	return analysis_ok;
}

/*---------------------------------------------------------------------------------------------------------------*/
static int16_t get_int16(uint8_t *buf, uint16_t offset)
{
	int16_t temp = 0;

	for (int8_t i = 1; i >= 0; i--)
	{
		temp <<= 8;
		temp |= buf[offset + i];
	}

	return temp;
}

static int32_t get_int32(uint8_t *buf, uint16_t offset)
{
	int32_t temp = 0;

	for (int8_t i = 3; i >= 0; i--)
	{
		temp <<= 8;
		temp |= buf[offset + i];
	}

	return temp;
}

static int64_t get_int64(uint8_t *buf, uint16_t offset)
{
	int64_t temp = 0;

	for (int8_t i = 7; i >= 0; i--)
	{
		temp <<= 8;
		temp |= buf[offset + i];
	}

	return temp;
}
