/****************************************************************
 * File:    Upgrade.cpp
 * 
 * Author:  PaceCat
 * 
 * Date:    2026-01-21
 * 
 * Description: 
 * This demo list how to upgrade lidar firmware include mcu and motor(auto recognition)
 * if upgrade finish,lidar will restart a few seconds
 * 
 * License: MIT
 * 
 * Notes:
 * 1.The callback function must not perform time-consuming operations, otherwise packet loss may occur
 * 2.use C++11 or later ; readerwriterqueue  3rdparty
 ****************************************************************/
#include "../sdk/pacecatlidarsdk.h"
#include"../sdk/global.h"

//集成使用以下数据需在以下回调函数中加锁同步
void LogDataCallback(uint32_t handle, const uint8_t dev_type, const char* data, size_t len) {
	if (data == nullptr) {
		return;
	}
	if (dev_type != MSG_CRITICAL)
	{
		char tmp[256]={0};
		memcpy(tmp,data,len);
		printf("ID::%d print level:%d msg:%s\n", handle, dev_type, tmp);
	}
}

int main()
{
	std::string lidar_ip = "192.168.158.98";
	int lidar_port = 6543;
	int listen_port = 7000;//不与数据监听端口相同即可
    //std::string  upgrade_file_path = "/home/pacecat/wangzn/ota/3d_m300_mcu_motor/test/LIDAR_M300-1206-20250822-192403_dual.lhr";
	//std::string  upgrade_file_path = "/home/pacecat/wangzn/ota/3d_m300_mcu_motor/test/LIDAR_M300-20250916-104712-dual.lhr";
	//std::string  upgrade_file_path = "/home/pacecat/wangzn/update_file/LDS-M300-E-20250924-150312.lhl";
	std::string  upgrade_file_path = "/home/pacecat/wangzn/update_file/LDS-M300-E-20250924-150312.lhl";
	int devID = PaceCatLidarSDK::getInstance()->AddLidarForUpgrade(lidar_ip,lidar_port,listen_port);
	PaceCatLidarSDK::getInstance()->SetLogDataCallback(devID, LogDataCallback, nullptr);
	bool ret = PaceCatLidarSDK::getInstance()->SetLidarUpgrade(devID, upgrade_file_path);
	printf("lidar upgrade %d\n", ret);
	// sleep(20);
	// bool ret2 = PaceCatLidarSDK::getInstance()->SetLidarUpgrade(devID, upgrade_file_path2);
	// printf("lidar upgrade2 %d\n", ret2);

	PaceCatLidarSDK::getInstance()->Uninit();
	return 0;
}