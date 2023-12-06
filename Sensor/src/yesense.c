//----
// @file yesense.c
// @author mask <beloved25177@126.com>
// @brief 这段代码很恶心，我也不想动，只好稍微一改，留个接口，里面的东西就给它尘封了吧
// @version 1.0
// @date 2023-11-26
//
// @copyright Copyright (c) 2023
//
//----

#include "yesense.h"

#include <stddef.h>

/*------------------------------------------------MARCOS define------------------------------------------------*/
#define PROTOCOL_FIRST_BYTE	 (unsigned char) 0x59
#define PROTOCOL_SECOND_BYTE (unsigned char) 0x53

#define PROTOCOL_FIRST_BYTE_POS	 0
#define PROTOCOL_SECOND_BYTE_POS 1

#define PROTOCOL_TID_LEN 2
#define PROTOCOL_MIN_LEN 7 /*header(2B) + tid(2B) + len(1B) + CK1(1B) + CK2(1B)*/

#define CRC_CALC_START_POS								 2
#define CRC_CALC_LEN(payload_len)					 ((payload_len) + 3) /*3 = tid(2B) + len(1B)*/
#define PROTOCOL_CRC_DATA_POS(payload_len) (CRC_CALC_START_POS + CRC_CALC_LEN(payload_len))

#define PAYLOAD_POS 5

#define SINGLE_DATA_BYTES 4

/*data id define*/
#define ACCEL_ID				(unsigned char) 0x10
#define ANGLE_ID				(unsigned char) 0x20
#define MAGNETIC_ID			(unsigned char) 0x30 /*归一化值*/
#define RAW_MAGNETIC_ID (unsigned char) 0x31 /*原始值*/
#define EULER_ID				(unsigned char) 0x40
#define QUATERNION_ID		(unsigned char) 0x41
#define UTC_ID					(unsigned char) 0x50
#define LOCATION_ID			(unsigned char) 0x60
#define SPEED_ID				(unsigned char) 0x70

/*length for specific data id*/
#define ACCEL_DATA_LEN				(unsigned char) 12
#define ANGLE_DATA_LEN				(unsigned char) 12
#define MAGNETIC_DATA_LEN			(unsigned char) 12
#define MAGNETIC_RAW_DATA_LEN (unsigned char) 12
#define EULER_DATA_LEN				(unsigned char) 12
#define QUATERNION_DATA_LEN		(unsigned char) 16
#define UTC_DATA_LEN					(unsigned char) 11
#define LOCATION_DATA_LEN			(unsigned char) 12
#define SPEED_DATA_LEN				(unsigned char) 12

/*factor for sensor data*/
#define NOT_MAG_DATA_FACTOR 0.000001f
#define MAG_RAW_DATA_FACTOR 0.001f

/*factor for gnss data*/
#define LONG_LAT_DATA_FACTOR 0.0000001
#define ALT_DATA_FACTOR			 0.001f
#define SPEED_DATA_FACTOR		 0.001f

/*------------------------------------------------Variables define------------------------------------------------*/
typedef enum { crc_err = -3, data_len_err = -2, para_err = -1, analysis_ok = 0, analysis_done = 1 } analysis_res_t;

typedef struct {
	unsigned char	 header1; /*0x59*/
	unsigned char	 header2; /*0x53*/
	unsigned short tid;			/*1 -- 60000*/
	unsigned char	 len;			/*length of payload, 0 -- 255*/
} output_data_header_t;

typedef struct {
	unsigned char data_id;
	unsigned char data_len;
} payload_data_t;
/*------------------------------------------------Functions declare------------------------------------------------*/
int get_signed_int(unsigned char* data);
int calc_checksum(unsigned char* data, unsigned short len, unsigned short* checksum);

/*-------------------------------------------------------------------------------------------------------------*/
unsigned char check_data_len_by_id(Yesense* yesense, unsigned char id, unsigned char len, unsigned char* data) {
	unsigned char ret					= 0xff;
	static float	yaw_start		= 0;
	static float	roll_start	= 0;
	static float	pitch_start = 0;

	switch (id) {
		case ACCEL_ID: {
			if (ACCEL_DATA_LEN == len) {
				ret							= (unsigned char) 0x1;
				yesense->accelx = get_signed_int(data) * NOT_MAG_DATA_FACTOR;
				yesense->accely = get_signed_int(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
				yesense->accelz = get_signed_int(data + (SINGLE_DATA_BYTES << 1)) * NOT_MAG_DATA_FACTOR;
			} else
				ret = (unsigned char) 0x00;
		} break;

		case ANGLE_ID: {
			if (ANGLE_DATA_LEN == len) {
				ret									= (unsigned char) 0x1;
				yesense->roll.dot		= get_signed_int(data) * NOT_MAG_DATA_FACTOR;
				yesense->pitch.dot	= get_signed_int(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
				yesense->yaw.dot		= get_signed_int(data + (SINGLE_DATA_BYTES << 1)) * NOT_MAG_DATA_FACTOR;
				yesense->yaw.dot	 *= DegToRad;
				yesense->roll.dot	 *= DegToRad;
				yesense->pitch.dot *= DegToRad;
				float dt						= (float) (GlobalTimer - yesense->timer) / 1000;
				yesense->timer			= GlobalTimer;
				if (dt != 0) {
					yesense->roll.ddot		 = (yesense->roll.dot - yesense->roll.lastdot) / dt;
					yesense->pitch.ddot		 = (yesense->pitch.dot - yesense->pitch.lastdot) / dt;
					yesense->yaw.ddot			 = (yesense->yaw.dot - yesense->yaw.lastdot) / dt;
					yesense->roll.lastdot	 = yesense->roll.dot;
					yesense->pitch.lastdot = yesense->pitch.dot;
					yesense->yaw.lastdot	 = yesense->yaw.dot;
				}
			} else
				ret = (unsigned char) 0x00;
		} break;

		case MAGNETIC_ID: {
			if (MAGNETIC_DATA_LEN == len) {
				ret						= (unsigned char) 0x1;
				yesense->magx = get_signed_int(data) * NOT_MAG_DATA_FACTOR;
				yesense->magy = get_signed_int(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
				yesense->magz = get_signed_int(data + (SINGLE_DATA_BYTES << 1)) * NOT_MAG_DATA_FACTOR;
			} else
				ret = (unsigned char) 0x00;
		} break;

		case RAW_MAGNETIC_ID: {
			if (MAGNETIC_RAW_DATA_LEN == len) {
				ret							 = (unsigned char) 0x1;
				yesense->rawMagx = get_signed_int(data) * MAG_RAW_DATA_FACTOR;
				yesense->rawMagy = get_signed_int(data + SINGLE_DATA_BYTES) * MAG_RAW_DATA_FACTOR;
				yesense->rawMagz = get_signed_int(data + (SINGLE_DATA_BYTES << 1)) * MAG_RAW_DATA_FACTOR;
			} else
				ret = (unsigned char) 0x00;
		} break;

		case EULER_ID: {
			if (EULER_DATA_LEN == len) {
				ret								 = (unsigned char) 0x1;
				yesense->pitch.now = get_signed_int(data) * NOT_MAG_DATA_FACTOR;
				yesense->roll.now	 = get_signed_int(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
				yesense->yaw.now	 = get_signed_int(data + (SINGLE_DATA_BYTES << 1)) * NOT_MAG_DATA_FACTOR;
				if (!yesense->init) {
					yaw_start			= yesense->yaw.now;
					pitch_start		= yesense->pitch.now;
					roll_start		= yesense->roll.now;
					yesense->init = true;
				}
				yesense->yaw.now	 -= yaw_start;
				yesense->roll.now	 -= roll_start;
				yesense->pitch.now -= pitch_start;
				yesense->yaw.now	 *= DegToRad;
				yesense->roll.now	 *= DegToRad;
				yesense->pitch.now *= DegToRad;
			} else
				ret = (unsigned char) 0x00;
		} break;
		case QUATERNION_ID: {
			if (QUATERNION_DATA_LEN == len) {
				ret												= (unsigned char) 0x1;
				yesense->quaternion_data0 = get_signed_int(data) * NOT_MAG_DATA_FACTOR;
				yesense->quaternion_data1 = get_signed_int(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
				yesense->quaternion_data2 = get_signed_int(data + (SINGLE_DATA_BYTES << 1)) * NOT_MAG_DATA_FACTOR;
				yesense->quaternion_data3 = get_signed_int(data + SINGLE_DATA_BYTES * 3) * NOT_MAG_DATA_FACTOR;
			} else
				ret = (unsigned char) 0x00;
		} break;
		default:
			break;
	}
	return ret;
}

/*--------------------------------------------------------------------------------------------------------------
 * 输出协议为：header1(0x59) + header2(0x53) + tid(2B) + payload_len(1B) + payload_data(Nbytes) + ck1(1B) + ck2(1B)
 * crc校验从TID开始到payload data的最后一个字节
 */
int yesenseAnalyze(Yesense* yesense, unsigned char* data, short len) {
	unsigned short payload_len		= 0;
	unsigned short check_sum			= 0;
	unsigned short pos						= 0;
	unsigned char	 ret						= 0xff;

	output_data_header_t* header	= NULL;
	payload_data_t*				payload = NULL;

	if (NULL == data || 0 >= len)
		return para_err;

	if (len < PROTOCOL_MIN_LEN)
		return data_len_err;

	/*judge protocol header*/
	if (PROTOCOL_FIRST_BYTE == data[PROTOCOL_FIRST_BYTE_POS] && PROTOCOL_SECOND_BYTE == data[PROTOCOL_SECOND_BYTE_POS]) {
		/*further check*/
		header			= (output_data_header_t*) data;
		payload_len = header->len;

		if (payload_len + PROTOCOL_MIN_LEN > len)
			return data_len_err;

		/*checksum*/
		calc_checksum(data + CRC_CALC_START_POS, CRC_CALC_LEN(payload_len), &check_sum);
		if (check_sum != *((unsigned short*) (data + PROTOCOL_CRC_DATA_POS(payload_len))))
			return crc_err;

		/*analysis payload data*/
		pos = PAYLOAD_POS;

		while (payload_len > 0) {
			payload = (payload_data_t*) (data + pos);
			ret			= check_data_len_by_id(yesense, payload->data_id, payload->data_len, (unsigned char*) payload + 2);
			if ((unsigned char) 0x01 == ret) {
				pos					+= payload->data_len + sizeof(payload_data_t);
				payload_len -= payload->data_len + sizeof(payload_data_t);
			} else {
				pos++;
				payload_len--;
			}
		}
		return analysis_ok;
	} else
		return analysis_done;
}

int get_signed_int(unsigned char* data) {
	int temp = 0;

	temp		 = (int) ((data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0]);

	return temp;
}

int calc_checksum(unsigned char* data, unsigned short len, unsigned short* checksum) {
	unsigned char	 check_a = 0;
	unsigned char	 check_b = 0;
	unsigned short i;

	if (NULL == data || 0 == len || NULL == checksum)
		return -1;

	for (i = 0; i < len; i++) {
		check_a += data[i];
		check_b += check_a;
	}

	*checksum = ((unsigned short) (check_b << 8) | check_a);
	return 0;
}

u8		HEAD_FLAG				 = 0;	 // 包头标志位
u8		RX_FLAG					 = 0;
short Data_len				 = 2;
u8		Data_Yesense[98] = { 0x59, 0x53 };
int		res							 = 0;
u8		n;
float yaw_last = 0, yaw_now = 0;

void G_output_infoSet(Yesense* yesense) {
	yaw_last = yaw_now;
	yaw_now	 = yesense->yaw.now;
	if (yaw_now - yaw_last < -100)	// 发生了突变
		n += 1;
	else if (yaw_now - yaw_last > 100)
		n -= 1;
	yesense->yaw.now += n * 360;
}

void yesenseReceiveHandler(Yesense* yesense, u8 temp) {
	if (RX_FLAG == 1) {
		Data_len++;
		Data_Yesense[Data_len - 1] = temp;
		if (Data_len > 97)	// 超出范围
		{
			Data_len = 2;
			RX_FLAG	 = 0;
		}
	}
	if (HEAD_FLAG == 1) {
		if (temp == 0x53)	 // 帧头2
		{
			Data_len	-= 2;
			res				 = yesenseAnalyze(yesense, Data_Yesense, Data_len);
			Data_len	 = 2;
			RX_FLAG		 = 1;
			HEAD_FLAG	 = 0;
			if (res == 0 || res == 1)
				G_output_infoSet(yesense);
		} else if (temp != 0x59)
			HEAD_FLAG = 0;
	}
	if (temp == 0x59)	 // 帧头1
		HEAD_FLAG = 1;
}

void yesenseInit(Yesense* yesense) {
	yesense->accelx	 = 0;
	yesense->accely	 = 0;
	yesense->accelz	 = 0;

	yesense->magx		 = 0;
	yesense->magy		 = 0;
	yesense->magz		 = 0;

	yesense->rawMagx = 0;
	yesense->rawMagy = 0;
	yesense->rawMagz = 0;

	datastructInit(&yesense->pitch, 0, 0, 0, 0);
	datastructInit(&yesense->roll, 0, 0, 0, 0);
	datastructInit(&yesense->yaw, 0, 0, 0, 0);

	yesense->quaternion_data0 = 0;
	yesense->quaternion_data1 = 0;
	yesense->quaternion_data2 = 0;
	yesense->quaternion_data3 = 0;

	yesense->init							= false;
	yesense->timer						= 0;
}
