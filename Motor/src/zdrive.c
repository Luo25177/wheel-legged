#include "zdrive.h"

typedef enum {
	// mod,0失能,1电流模式,2速度模式,3位置模式,4测试模式,5电阻电感校准,6编码器线性补偿,7编码器偏移校准,8VK校准,9保存配置,10擦除配置,11清除错误,12刹车
	Zdrive_Disable = 0,
	Zdrive_Current,
	Zdrive_Speed,
	Zdrive_Position,
	Zdrive_Test,
	Zdrive_RVCalibration,
	Zdrive_EncoderLineCalibration,
	Zdrive_EncoudeOffsetCalibration,
	Zdrive_VKCalibration,
	Zdrive_SaveSetting,
	Zdrive_EraseSetting,
	Zdrive_ClearErr,	// 擦除错误
	Zdrive_Brake
} Zdrive_Mode;
