/****************************************************************
 * File:    pacecatlidarsdk.cpp
 * 
 * Author:  PaceCat
 * 
 * Date:    2026-01-21
 * 
 * Description: 
 * This class is core module
 * include  lidar  pointcloud/imu recvform thread,cmd talk thread,playback thread,heatbeat thread
 * 
 * License: MIT
 * 
 * Notes:
 * 1.supports multiple lidars, but pay attention to bandwidth performance
 * 2.use C++11 or later ; readerwriterqueue  3rdparty
 ****************************************************************/

#pragma once

#include<vector>
#include<queue>
#include<thread>
#include<mutex>
#include<string>
#ifdef _WIN32
#pragma warning(disable : 4996)
#pragma warning(disable : 4244)
#include <winsock2.h>
#include <iphlpapi.h>
#pragma comment(lib, "iphlpapi.lib")
#pragma comment(lib, "ws2_32.lib")
typedef uint32_t in_addr_t;
#else
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <arpa/inet.h>
#endif
#include "../3rdparty/readerwriterqueue/readerwriterqueue.h"
#include"protocol.h"
#include"global.h"
#include"event.h"

#define M_SERIES_SDKVERSION "V1.7.1_2026012101"
typedef struct
{
	std::string uuid;
	std::string model;
	std::string lidarip;
	std::string lidarmask;
	std::string lidargateway;
	uint16_t lidarport;
	std::string uploadip;
	uint16_t uploadport;
	uint8_t uploadfix;

}BaseInfo;

typedef struct
{
	std::string mcu_ver;
	std::string motor_ver;
	std::string software_ver;
}VersionInfo;

typedef struct
{
	std::string sn;
	std::string ip;
	int port;
	uint64_t timestamp;
	int flag;//during a time and not recv heart package
	uint16_t motor_rpm; // 0.1
	uint16_t mirror_rpm;//1
	uint16_t temperature; // 0.1
	uint16_t voltage;	  // 0.001
	bool isonline;
}ConnectInfo;
typedef struct
{
	std::vector<ConnectInfo>lidars;
	int code;
	std::string value;
	bool isrun;
	std::string adapter;
}HeartInfo;

struct CmdTask
{
    uint64_t send_timestamp;//发送的时间戳
    uint8_t tried;//已经尝试次数
    std::string cmd;//测试指令
	int cmd_type;//指令类型    C_PACK,S_PACK,GS_PACK
	uint16_t rand;//随机码
	uint8_t is_inside;//是否是内置的命令
};
struct CmdTaskList
{
	uint8_t max_waittime;//最大等待时间 单位:秒
	uint8_t max_try_count;//最大重试次数
	std::queue<CmdTask>cmdtask;//任务列表
};
struct StatisticsInfo
{
	uint32_t zero_point_num;
	uint32_t distance_close_num;
	uint32_t sum_point_num;
	uint32_t filter_num;
};


enum LidarState
{
	OFFLINE = 0,
	ONLINE,
	QUIT
};
enum LidarAction
{
	NONE=0,
	WAIT,
	FINISH,
	START,
	STOP,
	RESTART,
	GET_PARAMS,
	GET_VERSION,
	SET_NETWORK,
	SET_UPLOAD_NETWORK,
	SET_UPLOAD_FIX,
	PTP_INIT,
	GET_NETERR,
	UPGRADE,
	CACHE_CLEAR,
	GET_DIRTY,
	GET_ENCODING_DISK,
	GET_ERRLIST,
	CLR_ERRLIST,
	GET_ADCINFO,
	CMD_TALK
};
enum LidarMsg
{
	MSG_DEBUG,
	MSG_WARM,
	MSG_ERROR,
	MSG_CRITICAL

};
enum CriticalMSG
{
	ERR_TEMPERATURE_HIGH=0,
	ERR_MOTOR_ZERO,
	ERR_MIRROR_ZERO,
	ERR_VOLTAGE_LOW,
	ERR_VOLTAGE_HIGH,
	ERR_MOTOR_NO_STABLE,
	ERR_MIRROR_NO_STABLE,
	ERR_DATA_ZERO
};

//运行配置 
struct RunConfig
{
	int ID;
	std::thread  thread_data;
	std::thread  thread_cmd;
	//std::thread  thread_pubCloud;
	//std::thread  thread_pubImu;
	LidarCloudPointCallback  cb_cloudpoint= nullptr;
	void *cloudpoint;
	LidarImuDataCallback cb_imudata = nullptr;
	void *imudata;
	LidarLogDataCallback cb_logdata = nullptr;
	void*logdata;
	LidarAlarmCallback cb_alarmdata = nullptr;
	void*alarmdata;
	uint8_t run_state;
	std::string lidar_ip;
	int lidar_port;
	int listen_port;
	std::vector<LidarCloudPointData> cloud_data;
	std::queue<IIM42652_FIFO_PACKET_16_ST> imu_data;
	IMUDrift  imu_drift;
	uint32_t frame_cnt;
	uint64_t frame_firstpoint_timestamp;  //everyframe  first point timestamp
	LidarAction action;
	std::string send_buf;
	int send_type;
	std::string recv_buf;
	int ptp_enable;
	bool cache_clear{false}; 
	ShadowsFilterParam sfp ;
	DirtyFilterParam   dfp;
	PointFilterParam   pfp;
	MatrixRotate_2 mr_2;
	StatisticsInfo stat_info;
	int frame_package_num;
	uint16_t package_num_idx{0};//当前帧计数
	int timemode;
	uint8_t rain;
	uint8_t echo_mode;
	std::string log_path;
	DeBugInfo debuginfo;
};
struct UserHeartInfo
{
	float motor_rpm; // 0.1
	uint16_t mirror_rpm;
	float temperature; // 0.1
	float voltage;	  // 0.001
	uint8_t isonline;//  offline/online
	uint64_t timestamp;
	int code;
	std::string value;

};

class PaceCatLidarSDK
{
public:
	static PaceCatLidarSDK *getInstance();
	static void deleteInstance();

	/*
	*    Bind the network card for communication
	*/
	void Init(std::string adapter);
	void Uninit();
	/*
	 *	callback function  get pointcloud data  imu data  logdata
	 */
	bool SetPointCloudCallback(int ID, LidarCloudPointCallback cb, void* client_data);
	bool SetImuDataCallback(int ID, LidarImuDataCallback cb, void* client_data);
	bool SetLogDataCallback(int ID, LidarLogDataCallback cb, void* client_data);
	bool SetAlarmDataCallback(int ID, LidarAlarmCallback cb, void* client_data);

	void WritePointCloud(int ID, const uint8_t dev_type, LidarPacketData *data);
	void WriteImuData( int ID, const uint8_t dev_type, LidarPacketData* data);
	void WriteLogData(int ID, const uint8_t dev_type, char* data, int len);
	void WriteAlarmData(int ID, const uint8_t dev_type, char* data, int len);

	/*
	 *	add lidar by lidar ip    lidar port    local listen port
	 */
	int AddLidar(ArgData argdata,ShadowsFilterParam sfp,DirtyFilterParam dfp,MatrixRotate_2 mr_2,PointFilterParam pfp);
	/*
	*   add lidar by  playback
	*/
	int AddLidarForPlayback(std::string logpath,int frame_rate);
	/*
	*   add lidar by  upgrade
	*/
	int AddLidarForUpgrade(std::string lidar_ip,int lidar_port,int listen_port);
	/*
	 *	connect lidar     send cmd/parse recvice data
	 */
	bool ConnectLidar(int ID,bool isplayback=false);
	/*
	 *	disconnect lidar,cancel listen lidar
	 */
	bool DisconnectLidar(int ID);

	/*
	 *	query connect lidar base info
	 */
	bool QueryBaseInfo(int ID, BaseInfo &info);

	/*
	 *	query connect lidar version
	 */
	bool QueryVersion(int ID, VersionInfo &info);

	/*
	 *	use by lidar heart  query lidar  state
	 */
	UserHeartInfo QueryDeviceState(int ID);

	/*
	 *	set lidar    ip  mask  gateway  receive port
	 */
	bool SetLidarNetWork(int ID,std::string ip, std::string mask, std::string gateway, uint16_t port);

	/*
	 *	set lidar    upload ip    upload port     fixed upload or not
	 */
	bool SetLidarUploadNetWork(int ID, std::string upload_ip, uint16_t upload_port);

	/*
	 *	set lidar    start work     stop work（sleep）  restart work(reboot)
	 */
	bool SetLidarAction(int ID, int action);
	/*
	 *	set lidar  firmware upgrade
	 */
	bool SetLidarUpgrade(int ID, std::string path);
	/*
	 *	set lidar  ptp init
	 */
	bool SetLidarPTP(int ID,bool ptp_enable);
	/*
	 *	set lidar  ptp init
	 */
	//bool SetLidarPTPInit(int ID);
	/*
	 *	query lidar  network err  netinfo
	 */
	bool QueryLidarNetWork(int ID,std::string& netinfo);
	/*
	 *	clear frame cache (Applied to situations   powered on or off, or rpm is unstable)
	 */
	void ClearFrameCache(int ID);
	/*
	 *	query lidar  dirty data total data,please use it at begin and error time
	 */
	bool QueryDirtyData(int ID,std::string &dirty_data);
	/*
	 *	query lidar  encoding disk info
	 */
	bool QueryMCUInfo(int ID,std::string &encoding_disk_info);

	/*
	 *	query lidar  errinfo list
	 */
	bool QueryLidarErrList(int ID,std::string& errlist);
	/*
	 *	clear lidar  errinfo list
	 */
	bool CleanLidarErrList(int ID);

	/*
	 *	get rain data
	 */
	bool QueryRainData(int ID,uint8_t&rain);
	/*
	 *	get echo mode
	 */
	bool QueryEchoMode(int ID,uint8_t&echo_mode);
	/*
	*   query high temperature  adc err code 
 	*/
 	bool QueryADCInfo(int ID,std::string& adcinfo);
	bool QueryTDCInfo(int ID,std::string& tdcinfo);

	bool QueryIMUInfo(int ID,std::string& imuinfo);
	bool SetIMUInfo(int ID,ImuInfo imuinfo);

	bool ReadCalib(int ID,std::string lidar_ip, int port);
	int QueryIDByIp(std::string ip);
	
private:
	void UDPDataThreadProc(int id);
	void ParseLogThreadProc(int id);
	void PlaybackThreadProc(int id);
	void HeartThreadProc(HeartInfo &heartinfo);
	//monitor other theard is working
	void WatchDogThreadProc(bool &isrun);

	void UDPCmdThreadProc(int id);

	int PackNetCmd(uint16_t type, uint16_t len, uint16_t sn, const void* buf, uint8_t* netbuf);
	int SendNetPack(int sock, uint16_t type, uint16_t len, const void* buf, char*ip, int port);
	void AddPacketToList(const BlueSeaLidarEthernetPacket* bluesea,  double& last_ang, std::vector<LidarCloudPointData>& tmp_filter, std::vector<double>& tmp_ang,RunConfig *cfg);
	double PacketToPoints(BlueSeaLidarSpherPoint bluesea, LidarCloudPointData& point,StatisticsInfo&stat_info);

	RunConfig* GetConfig(int ID);
	bool FirmwareUpgrade(int ID,std::string  ip, int port,int listenport,std::string path);
	std::string QuerySysEvent(char*buf,int len);

	bool GetImuFSSEL(int AccelScale,int GyroScale,float &AccelScale2,float &GyroScale2);
	void AnomalyDetection(int ID,DeBugInfo& debuginfo,StatisticsInfo&stat_info,uint64_t timestamp);
private:
	static PaceCatLidarSDK *m_sdk;
	PaceCatLidarSDK();
	~PaceCatLidarSDK();

	int m_idx;
	std::vector<RunConfig*> m_lidars;
	std::thread m_heartthread;
	std::thread m_watchdog_thread;
	HeartInfo m_heartinfo;
	moodycamel::ReaderWriterQueue<std::string> m_log_queue;

	    //检测是否因为回调卡主导致的打印异常
    bool m_datathread_is_block{false};
    bool m_cmdthread_is_block{false};
    bool m_heartthread_is_block{false};
};








