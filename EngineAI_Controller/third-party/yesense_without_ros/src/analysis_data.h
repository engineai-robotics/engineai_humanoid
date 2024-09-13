#ifndef ANALYSIS_DATA_H
#define ANALYSIS_DATA_H

#ifdef __cplusplus
extern "C"
{
#endif

/*------------------------------------------------Macros define------------------------------------------------*/
#define PROTOCOL_FIRST_BYTE			(unsigned char)0x59
#define PROTOCOL_SECOND_BYTE		(unsigned char)0x53
#define PROTOCOL_MIN_LEN			(unsigned short)7	/*header(2B) + tid(2B) + len(1B) + CK1(1B) + CK2(1B)*/

#define YIS_RESPONSE_CMD_DATA_POS	(unsigned short)5

/*------------------------------------------------Type define--------------------------------------------------*/
typedef enum
{
	crc_err = -3,
	data_len_err = -2,
	para_err = -1,
	analysis_ok = 0,
	analysis_done = 1
}analysis_res_t;

#pragma pack(1)

typedef struct
{
	unsigned char header1;	/*0x59*/
	unsigned char header2;	/*0x53*/
	unsigned short tid;		/*1 -- 60000*/
	unsigned char len;		/*length of payload, 0 -- 255*/
}output_data_header_t;

typedef struct
{
	unsigned char data_id;
	unsigned char data_len;
}payload_data_t;

typedef struct
{
	unsigned char  type;			/*command type*/	
	unsigned short op_id:3;			/*operation*/
	unsigned short len:13;			/*length*/
	unsigned char  response_need:1;	/**/
	unsigned char  response_found:1;
	unsigned char  response_recv_done:1;
	unsigned char  resv:5;
}yis_cmd_response_t;

#pragma pack()

typedef enum ClassType{
	PRODUCT_INFOMATION       = 0x00,
	UART_BAUDRATE            = 0x02,
	OUTPUT_FREEQUENCY        = 0x03,
	OUTPUT_CONTENT           = 0x04,
	CALIBRATION_PARAM_SET    = 0x05,
	MODE_SETTING             = 0x4D,
	NEMA0183_OUTPUT_CONTENT  = 0x4E
}ClassType;

typedef enum ClassID{
	QUERY_STATE     = 0x00,
	SET_STATE_MEM   = 0x01,
	SET_STATE_FLASH = 0x02
}ClassID;

typedef enum BaudRate{
	BAUDRATE_9600 = 0x01,
	BAUDRATE_38400,
	BAUDRATE_115200,
	BAUDRATE_460800,
	BAUDRATE_921600,
	BAUDRATE_19200,
	BAUDRATE_57600,
	BAUDRATE_78600,
	BAUDRATE_230400,
}BaudRate;

typedef enum Freequency{
	FREEQUENCY_1_HZ = 0x01,
	FREEQUENCY_2_HZ,
	FREEQUENCY_5_HZ,
	FREEQUENCY_10_HZ,
	FREEQUENCY_20_HZ,
	FREEQUENCY_25_HZ,
	FREEQUENCY_50_HZ,
	FREEQUENCY_100_HZ,
	FREEQUENCY_200_HZ,
	FREEQUENCY_250_HZ,
	FREEQUENCY_500_HZ,
	FREEQUENCY_1000_HZ
}Freequency;

typedef enum OUTPUT_CONTENT_INFO{
	SPEED_ENABLE                    = 0x00,
	LOCATION_ENABLE,
	UTC_ENABLE,
	QUATERNION_ENABLE,
	EULER_ENABLE,
	MAGNETIC_ENABLE,
	ANGULAR_VELICITY_ENABLE,
	ACCEL_INCREAMENT_ENABLE,
	VELICITY_INCREAMENT_ENABLE,
	QUATERNION_INCREAMENT_ENABLE,
	IMU_TEMP_ENABLE,
	SECOND_IMU_ANGLE_ENABLE,
	SECOND_IMU_ACCEL_ENABLE,
	SECOND_IMU_TEMP_ENABLE,
	FREE_ACCEL_ENABLE,
	TIMESTAMP_ENABLE	
}OUTPUT_CONTENT_INFO;

#pragma pack(1)
typedef struct 
{
	struct
	{
		unsigned int ms;		
		unsigned short year;
		unsigned char month;
		unsigned char  date;
		unsigned char  hour;
		unsigned char  min;
		unsigned char  sec;
	} utc_time;

	struct 
	{
		double latitude;					/*unit: deg*/
		double longtidue;					/*unit: deg*/
		float altidue;						/*unit: m*/
	} location;

	struct 
	{
		float lon;
		float lat;
		float alt;
	} location_error;

	struct 
	{	
		float vel_e;						/*unit: m/s */
		float vel_n;
		float vel_u;
	}vel;

	float ground_speed;
	float yaw;
	unsigned char  status;
	unsigned char  star_cnt;
	float p_dop;
	unsigned char  site_id;
}imu_gnss_data_t;

typedef struct
{
	unsigned int sample_timestamp;		/*unit: us*/
	unsigned int out_sync_timestamp;	/*unit: us*/ 
	unsigned short tid;					/*range 0 - 60000 */

	float imu_temp;                		/*unit: °C*/    
	
	float accel_x;						/*unit: m/s2*/
	float accel_y;
	float accel_z;

	float angle_x;						/*unit: ° (deg)/s*/
	float angle_y;
	float angle_z;

	float mag_x;						/*unit: 归一化值*/
	float mag_y;
	float mag_z;

	float raw_mag_x;					/*unit: mGauss*/
	float raw_mag_y;
	float raw_mag_z;
	
	float pitch;						/*unit: ° (deg)*/
	float roll;
	float yaw;
	
	float quaternion_data0;
	float quaternion_data1;	
	float quaternion_data2;
	float quaternion_data3;
	
	uint8_t fusion_status;

	imu_gnss_data_t gnss; /* master GNSS */

	struct
	{
		float dual_ant_yaw;
		float dual_ant_yaw_error;
		float dual_ant_baseline_len;
	} gnss_slave; /* slave GNSS */

}protocol_info_t;
#pragma pack()

/*----------------------------------------------------------------------------------------*/
int analysis_data(unsigned char *data, short len, protocol_info_t *info, yis_cmd_response_t *cmd);
int calc_checksum(unsigned char *data, unsigned short len, unsigned short *checksum);
int parse_data_by_id(payload_data_t header, unsigned char *data, protocol_info_t *info);

#ifdef __cplusplus
}
#endif

#endif
