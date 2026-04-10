#include <fstream>
#include <chrono>
#include <iostream>
#include <ctime>
#include <sstream>
#include "global.h"
#include "pacecatlidarsdk.h"
#include "upgrade.h"

#ifdef __unix__
#include "playback.h"
#endif
using namespace moodycamel;

PaceCatLidarSDK *PaceCatLidarSDK::m_sdk = new (std::nothrow) PaceCatLidarSDK();
PaceCatLidarSDK *PaceCatLidarSDK::getInstance()
{
	return m_sdk;
}

void PaceCatLidarSDK::deleteInstance()
{
	if (m_sdk)
	{
		delete m_sdk;
		m_sdk = NULL;
	}
}
PaceCatLidarSDK::PaceCatLidarSDK()
{
	m_heartinfo.isrun = false;
}

PaceCatLidarSDK::~PaceCatLidarSDK()
{
	Uninit();
}
bool PaceCatLidarSDK::SetPointCloudCallback(int ID, LidarCloudPointCallback cb, void *client_data)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	lidar->cb_cloudpoint = cb;
	lidar->cloudpoint = client_data;
	return true;
}
bool PaceCatLidarSDK::SetImuDataCallback(int ID, LidarImuDataCallback cb, void *client_data)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	lidar->cb_imudata = cb;
	lidar->imudata = (char *)client_data;
	return true;
}
bool PaceCatLidarSDK::SetLogDataCallback(int ID, LidarLogDataCallback cb, void *client_data)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	lidar->cb_logdata = cb;
	lidar->logdata = (char *)client_data;
	return true;
}
bool PaceCatLidarSDK::SetAlarmDataCallback(int ID, LidarAlarmCallback cb, void *client_data)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	lidar->cb_alarmdata = cb;
	lidar->alarmdata = (char *)client_data;
	return true;
}

void PaceCatLidarSDK::WritePointCloud(int ID, const uint8_t dev_type, LidarPacketData *data)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return;

	if (lidar->cb_cloudpoint != nullptr)
		lidar->cb_cloudpoint(ID, dev_type, data, lidar->cloudpoint);
}

void PaceCatLidarSDK::WriteImuData(int ID, const uint8_t dev_type, LidarPacketData *data)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return;
	if (lidar->cb_imudata != nullptr)
		lidar->cb_imudata(ID, dev_type, data, lidar->imudata);
}

void PaceCatLidarSDK::WriteLogData(int ID, const uint8_t dev_type, char *data, size_t len)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return;

	if (lidar->cb_logdata != nullptr)
	{
		std::ostringstream logbuf;
		logbuf << "time:" << SystemAPI::getCurrentTime() << std::string(data, len);
		lidar->cb_logdata(ID, dev_type, logbuf.str().c_str(), logbuf.str().size());
	}
}
void PaceCatLidarSDK::WriteAlarmData(int ID, const uint8_t dev_type, char *data, size_t len)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return;

	if (lidar->cb_alarmdata != nullptr)
		lidar->cb_alarmdata(ID, dev_type, data, len);
}
void PaceCatLidarSDK::Init(std::string adapter)
{
	if (!m_heartinfo.isrun)
	{
		m_heartinfo.code = 0;
		m_heartinfo.isrun = true;
		m_heartinfo.adapter = adapter;
		m_heartthread = std::thread(&PaceCatLidarSDK::HeartThreadProc, PaceCatLidarSDK::getInstance(), std::ref(m_heartinfo));
		m_heartthread.detach();
		m_watchdog_thread = std::thread(&PaceCatLidarSDK::WatchDogThreadProc, PaceCatLidarSDK::getInstance(), std::ref(m_heartinfo.isrun));
		m_watchdog_thread.detach();
	}
}
void PaceCatLidarSDK::Uninit()
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		m_lidars.at(i)->run_state = QUIT;
	}
	m_heartinfo.isrun = false;
}
int PaceCatLidarSDK::AddLidar(ArgData argdata, ShadowsFilterParam sfp, DirtyFilterParam dfp, MatrixRotate_2 mr_2, PointFilterParam pfp)
{
	RunConfig *cfg = new RunConfig;
	cfg->lidar_ip = argdata.lidar_ip;
	cfg->lidar_port = argdata.lidar_port;
	cfg->listen_port = argdata.listen_port;
	cfg->ID = static_cast<int>(m_lidars.size());
	cfg->run_state = ONLINE;
	cfg->frame_cnt = 0;
	cfg->cb_cloudpoint = NULL;
	cfg->cb_imudata = NULL;
	cfg->cb_logdata = NULL;
	cfg->frame_firstpoint_timestamp = 0;
	cfg->ptp_enable = argdata.ptp_enable;
	cfg->sfp = sfp;
	cfg->dfp = dfp;
	cfg->pfp = pfp;
	cfg->frame_package_num = argdata.frame_package_num;
	cfg->timemode = argdata.timemode;
	cfg->mr_2 = mr_2;
	cfg->stat_info = StatisticsInfo{0, 0, 0, 0};
	m_lidars.push_back(cfg);
	return cfg->ID;
}
#ifdef __unix__
int PaceCatLidarSDK::AddLidarForPlayback(std::string logpath, int frame_rate)
{
	RunConfig *cfg = new RunConfig;
	cfg->log_path = logpath;
	cfg->frame_package_num = frame_rate;
	cfg->ID = static_cast<int>(m_lidars.size());
	cfg->run_state = OFFLINE;
	m_lidars.push_back(cfg);
	return cfg->ID;
}
#endif
int PaceCatLidarSDK::AddLidarForUpgrade(std::string lidar_ip, int lidar_port, int listen_port)
{
	RunConfig *cfg = new RunConfig;
	cfg->lidar_ip = lidar_ip;
	cfg->lidar_port = lidar_port;
	cfg->listen_port = listen_port;
	cfg->ID = static_cast<int>(m_lidars.size());
	cfg->run_state = OFFLINE;
	m_lidars.push_back(cfg);
	return cfg->ID;
}
bool PaceCatLidarSDK::ConnectLidar(int ID, bool isplayback)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	if (!isplayback)
	{
		char log_buf[128] = {0};
		sprintf(log_buf, "start read calib");
		WriteLogData(ID, MSG_WARM, log_buf, strlen(log_buf));
		// 读取标定文件
		bool result = false;
		for (int i = 5; i >= 0; i--)
		{
			result = PaceCatLidarSDK::getInstance()->ReadCalib(ID, lidar->lidar_ip, lidar->lidar_port);
			if (result)
				break;
		}

		sprintf(log_buf, "read calib result:%s", result ? RECV_OK : RECV_NG);
		WriteLogData(ID, MSG_WARM, log_buf, strlen(log_buf));
		if (!result)
			return false;

		lidar->cmd_fd = SystemAPI::open_socket_port();
		if (lidar->cmd_fd <= 0)
		{
			std::string err = "create cmd socket failed";
			WriteLogData(ID, MSG_ERROR, (char *)err.c_str(), err.size());
			return false;
		}
		std::string response;
		if (udp_talk_pack(lidar->cmd_fd, lidar->lidar_ip, lidar->lidar_port, "LEVENTRH", C_PACK, response, 500, 5))
		{
			std::string sysevent = QuerySysEvent((char *)response.c_str(), response.size());
			WriteLogData(ID, MSG_WARM, (char *)sysevent.c_str(), sysevent.size());
		}

		if (lidar->ptp_enable >= 0)
		{
			char ptpcmd[16] = {0};
			sprintf(ptpcmd, "LSPTP:%dH", lidar->ptp_enable);
			if (udp_talk_pack(lidar->cmd_fd, lidar->lidar_ip, lidar->lidar_port, ptpcmd, S_PACK, response, 500, 5))
			{
				WriteLogData(ID, MSG_WARM, (char *)response.c_str(), response.size());
			}
		}

		lidar->thread_data = std::thread(&PaceCatLidarSDK::UDPDataThreadProc, PaceCatLidarSDK::getInstance(), ID, std::ref(m_heartinfo));
		lidar->thread_data.detach();
	}

#ifdef __unix__
	else
	{
		std::thread parselog_thread = std::thread(&PaceCatLidarSDK::ParseLogThreadProc, PaceCatLidarSDK::getInstance(), ID);
		parselog_thread.detach();
		std::thread playback_thread = std::thread(&PaceCatLidarSDK::PlaybackThreadProc, PaceCatLidarSDK::getInstance(), ID);
		playback_thread.detach();
	}
#endif
	return true;
}
bool PaceCatLidarSDK::DisconnectLidar(int ID)
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			m_lidars.at(i)->run_state = QUIT;
			SystemAPI::closefd(m_lidars.at(i)->cmd_fd, true);
			printf("close cmd socket %d\n", m_lidars.at(i)->cmd_fd);
			m_lidars.at(i)->cmd_fd = -1;
			return true;
		}
	}
	return false;
}

bool PaceCatLidarSDK::QueryBaseInfo(int ID, BaseInfo &info)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	std::string response;
	if (udp_talk_pack(lidar->cmd_fd, lidar->lidar_ip, lidar->lidar_port, "xxxxxx", GS_PACK, response, 500, 5))
	{
		if (response.size() == sizeof(EEpromV101))
		{
			EEpromV101 *eepromv101 = (EEpromV101 *)response.c_str();
			info.uuid = BaseAPI::stringfilter((char *)eepromv101->dev_sn, 20);
			info.model = BaseAPI::stringfilter((char *)eepromv101->dev_type, 16);

			char tmp_IPv4[16] = {0};
			char tmp_mask[16] = {0};
			char tmp_gateway[16] = {0};
			char tmp_srv_ip[16] = {0};
			sprintf(tmp_IPv4, "%d.%d.%d.%d", eepromv101->IPv4[0], eepromv101->IPv4[1], eepromv101->IPv4[2],
					eepromv101->IPv4[3]);
			sprintf(tmp_mask, "%d.%d.%d.%d", eepromv101->mask[0], eepromv101->mask[1], eepromv101->mask[2],
					eepromv101->mask[3]);
			sprintf(tmp_gateway, "%d.%d.%d.%d", eepromv101->gateway[0], eepromv101->gateway[1], eepromv101->gateway[2],
					eepromv101->gateway[3]);
			sprintf(tmp_srv_ip, "%d.%d.%d.%d", eepromv101->srv_ip[0], eepromv101->srv_ip[1], eepromv101->srv_ip[2],
					eepromv101->srv_ip[3]);

			info.lidarip = tmp_IPv4;
			info.lidarmask = tmp_mask;
			info.lidargateway = tmp_gateway;
			info.lidarport = eepromv101->local_port;
			info.uploadip = tmp_srv_ip;
			info.uploadport = eepromv101->srv_port;
			info.uploadfix = eepromv101->target_fixed;
			return true;
		}
	}
	return false;
}
bool PaceCatLidarSDK::QueryVersion(int ID, VersionInfo &info)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	std::string response;
	if (udp_talk_pack(lidar->cmd_fd, lidar->lidar_ip, lidar->lidar_port, "LXVERH", C_PACK, response, 500, 5))
	{
		if (response == RECV_NG)
		{
			return false;
		}
		std::string tmp2 = response;
		size_t idx = tmp2.find(0x0d);
		size_t idx2 = tmp2.find(0x3a);
		if (idx == std::string::npos || idx == std::string::npos)
			return false;
		info.mcu_ver = tmp2.substr(idx2 + 1, idx - idx2 - 1);
		tmp2 = tmp2.substr(idx + 2);
		idx = tmp2.find(0x0d);
		idx2 = tmp2.find(0x3a);
		if (idx == std::string::npos || idx == std::string::npos)
			return false;
		info.motor_ver = tmp2.substr(idx2 + 1, idx - idx2 - 1);
		info.software_ver = M_SERIES_SDKVERSION;
		return true;
	}
	return false;
}
UserHeartInfo PaceCatLidarSDK::QueryDeviceState(int ID)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return UserHeartInfo{0, 0, 0, 0, 0, 0, 0, ""};

	std::string ip = lidar->lidar_ip;
	std::lock_guard<std::mutex> lock(m_heartinfo.heart_mutex);
	for (size_t i = 0; i < m_heartinfo.lidars.size(); i++)
	{
		std::string tmpip = m_heartinfo.lidars.at(i).ip;
		if (ip == tmpip)
		{
			UserHeartInfo heartinfo;
			heartinfo.mirror_rpm = m_heartinfo.lidars.at(i).mirror_rpm;
			heartinfo.motor_rpm = m_heartinfo.lidars.at(i).motor_rpm / 10.0f;
			heartinfo.temperature = m_heartinfo.lidars.at(i).temperature / 10.0f;
			heartinfo.voltage = m_heartinfo.lidars.at(i).voltage / 1000.0f;
			heartinfo.isonline = m_heartinfo.lidars.at(i).isonline;
			heartinfo.timestamp = m_heartinfo.lidars.at(i).timestamp;
			heartinfo.code = m_heartinfo.code;
			heartinfo.value = m_heartinfo.value;
			return heartinfo;
		}
	}
	return UserHeartInfo{0, 0, 0, 0, 0, 0, m_heartinfo.code, m_heartinfo.value};
}

bool PaceCatLidarSDK::SetLidarNetWork(int ID, std::string ip, std::string mask, std::string gateway, uint16_t port)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	char result[64] = {0};
	// 对传入的格式校验
	if (!BaseAPI::checkAndMerge(1, (char *)ip.c_str(), (char *)mask.c_str(), (char *)gateway.c_str(), port, result))
	{
		return false;
	}
	char tmp[128] = {0};
	sprintf(tmp, "LSUDP:%sH", result);
	std::string response;
	if (udp_talk_pack(lidar->cmd_fd, lidar->lidar_ip, lidar->lidar_port, tmp, S_PACK, response, 500, 1))
	{
		if (response.find(RECV_NG) != std::string::npos)
		{
			return false;
		}
		if (response.find(RECV_OK) != std::string::npos)
		{
			lidar->lidar_ip = ip;
			lidar->lidar_port = port;
			return true;
		}
	}
	return true;
}

bool PaceCatLidarSDK::SetLidarUploadNetWork(int ID, std::string upload_ip, uint16_t upload_port)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	char result[50] = {0};
	// 对传入的格式校验
	if (!BaseAPI::checkAndMerge(0, (char *)upload_ip.c_str(), (char *)"", (char *)"", upload_port, result))
	{
		return false;
	}
	char tmp[64] = {0};
	sprintf(tmp, "LSDST:%sH", result);
	std::string response;
	if (udp_talk_pack(lidar->cmd_fd, lidar->lidar_ip, lidar->lidar_port, tmp, S_PACK, response, 500, 5))
	{
		if (response.find(RECV_NG) != std::string::npos)
		{
			return false;
		}
		if (response.find(RECV_OK) != std::string::npos)
		{
			return true;
		}
	}
	return false;
}
bool PaceCatLidarSDK::SetLidarUploadFix(int ID, bool isfix)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	char tmp[64] = {0};
	sprintf(tmp, "LSTFX:%dH", isfix);
	std::string response;
	if (udp_talk_pack(lidar->cmd_fd, lidar->lidar_ip, lidar->lidar_port, tmp, S_PACK, response, 500, 5))
	{
		if (response.find(RECV_NG) != std::string::npos)
		{
			return false;
		}
		if (response.find(RECV_OK) != std::string::npos)
		{
			return true;
		}
	}
	return false;
}


bool PaceCatLidarSDK::SetLidarAction(int ID, int action)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	std::string command;
	if (action == START)
		command = "LSTARH";
	else if (action == STOP)
		command = "LSTOPH";
	else if (action == RESTART)
		command = "LRESTH";
	else
		return false;

	std::string response;
	if (udp_talk_pack(lidar->cmd_fd, lidar->lidar_ip, lidar->lidar_port, command, C_PACK, response, 500, 5))
	{
		if (response.find(RECV_NG) != std::string::npos)
			return false;
		if (response.find(RECV_OK) != std::string::npos)
			return true;
	}
	return false;
}

bool PaceCatLidarSDK::SetLidarUpgrade(int ID, std::string path)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	std::ifstream f(path.c_str());
	if (!f.good())
		return false;

	return PaceCatLidarSDK::getInstance()->FirmwareUpgrade(ID, lidar->lidar_ip.c_str(), lidar->lidar_port, lidar->listen_port, path);
}
bool PaceCatLidarSDK::SetLidarPTP(int ID, bool ptp_enable)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	char tmp[16] = {0};
	sprintf(tmp, "LSPTP:%cH", '0' + ptp_enable);
	std::string response;
	if (udp_talk_pack(lidar->cmd_fd, lidar->lidar_ip, lidar->lidar_port, tmp, S_PACK, response, 500, 5))
	{
		if (response.find(RECV_NG) != std::string::npos)
		{
			return false;
		}
		if (response.find(RECV_OK) != std::string::npos)
		{
			return true;
		}
	}

	return false;
}
// bool PaceCatLidarSDK::SetLidarPTPInit(int ID)
// {
// 	RunConfig *lidar = GetConfig(ID);
// 	if (lidar == nullptr)
// 		return false;

// 	char tmp[64] = {0};
// 	sprintf(tmp, "LPTPINITH");
// 	lidar->send_buf = std::string(tmp,strlen(tmp));
// 	lidar->send_type = C_PACK;
// 	lidar->action = LidarAction::CMD_TALK;
// 	int index = CMD_REPEAT;
// 	while (lidar->action != LidarAction::FINISH && index > 0)
// 	{
// 		std::this_thread::sleep_for(std::chrono::milliseconds(100));
// 		index--;
// 	}
// 	if (lidar->action == LidarAction::FINISH)
// 	{
// 		lidar->action = LidarAction::NONE;
// 		// printf("%s %d\n",lidar->recv_buf.c_str(),lidar->recv_len);
// 		std::string recv_buf = lidar->recv_buf;
// 		if (recv_buf.find("PTP sync") != std::string::npos)
// 			return true;
// 	}

// 	return false;
// }
bool PaceCatLidarSDK::QueryLidarNetWork(int ID, std::string &netinfo)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	if (udp_talk_pack(lidar->cmd_fd, lidar->lidar_ip, lidar->lidar_port, "LNETRRH", C_PACK, netinfo, 500, 5))
	{
		if (netinfo.find("Boot:") != std::string::npos)
			return true;
	}
	return false;
}
void PaceCatLidarSDK::ClearFrameCache(int ID)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return;

	lidar->cache_clear = true;
}

bool PaceCatLidarSDK::QueryDirtyData(int ID, std::string &dirty_data)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	if (udp_talk_pack(lidar->cmd_fd, lidar->lidar_ip, lidar->lidar_port, "LURERH", C_PACK, dirty_data, 500, 5))
		return true;

	return false;
}
bool PaceCatLidarSDK::QueryMCUInfo(int ID, std::string &mcu_data)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	if (udp_talk_pack(lidar->cmd_fd, lidar->lidar_ip, lidar->lidar_port, "LQLVXH", C_PACK, mcu_data, 500, 5))
		return true;
	return false;
}

bool PaceCatLidarSDK::QueryLidarErrList(int ID, std::string &errlist)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	std::string response;
	if (udp_talk_pack(lidar->cmd_fd, lidar->lidar_ip, lidar->lidar_port, "LEVENTRH", C_PACK, response, 500, 5))
	{
		errlist = QuerySysEvent((char *)response.c_str(), response.size());
		return true;
	}
	return false;
}

bool PaceCatLidarSDK::QueryRainData(int ID, uint8_t &rain)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	rain = lidar->rain;
	return true;
}
bool PaceCatLidarSDK::QueryEchoMode(int ID, uint8_t &echo_mode)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	echo_mode = lidar->echo_mode;
	return true;
}

bool PaceCatLidarSDK::QueryADCInfo(int ID, std::string &adcinfo)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	if (udp_talk_pack(lidar->cmd_fd, lidar->lidar_ip, lidar->lidar_port, "LMBADCH", C_PACK, adcinfo, 500, 5))
	{
		return true;
	}
	return false;
}
bool PaceCatLidarSDK::QueryTDCInfo(int ID, std::string &tdcinfo)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	if (udp_talk_pack(lidar->cmd_fd, lidar->lidar_ip, lidar->lidar_port, "LQTDCH", C_PACK, tdcinfo, 500, 5))
	{
		return true;
	}
	return false;
}
bool PaceCatLidarSDK::QueryIMUInfo(int ID, std::string &imuinfo)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;
	if (udp_talk_pack(lidar->cmd_fd, lidar->lidar_ip, lidar->lidar_port, "LQIMUCFGH", C_PACK, imuinfo, 500, 5))
	{
		return true;
	}
	return false;
}
bool PaceCatLidarSDK::SetIMUInfo(int ID, ImuInfo imuinfo)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	char imu_buf[255] = {0};
	sprintf(imu_buf, "LSIMU:ACCE %x,%x,%x;GYRO %x,%x,%xH",
			imuinfo.acc_range, imuinfo.acc_ord, imuinfo.acc_filter_level,
			imuinfo.gyro_range, imuinfo.gyro_ord, imuinfo.gyro_filter_level);

	std::string response;
	if (udp_talk_pack(lidar->cmd_fd, lidar->lidar_ip, lidar->lidar_port, imu_buf, S_PACK, response, 500, 5))
	{
		if (response.find(RECV_NG) != std::string::npos)
		{
			return false;
		}
		if (response.find(RECV_OK) != std::string::npos)
		{
			return true;
		}
	}
	return false;
}

int PaceCatLidarSDK::QueryIDByIp(std::string ip)
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->lidar_ip == ip && m_lidars.at(i)->run_state != QUIT)
		{
			return m_lidars.at(i)->ID;
		}
	}
	return -1;
}

RunConfig *PaceCatLidarSDK::GetConfig(int ID)
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			return m_lidars.at(i);
		}
	}
	return nullptr;
}
std::string PaceCatLidarSDK::QuerySysEvent(char *buf, size_t len)
{
	std::ostringstream result;
	SYS_EVENT_LOG *syseventlog = (SYS_EVENT_LOG *)(buf);
	result << "Current_Time_10mSec:" << (int)syseventlog->Current_Event_Index << " "
		   << "Complement:" << syseventlog->Current_Time_10mSec_Complement << " "
		   << "Boot_Count:" << (int)syseventlog->Boot_Count << " "
		   << "Event_Count:" << (int)syseventlog->Event_Count << " "
		   << "Current_Event_Index:" << (int)syseventlog->Current_Event_Index;

	int sysevent_num = syseventlog->Event_Count >= 128 ? MAX_EVENT_COUNT : syseventlog->Event_Count;
	for (int i = 0; i < sysevent_num; i++)
	{
		EVENT_LOG *eventlog = &syseventlog->Event[i];
		result << "\r\n"
			   << std::to_string(eventlog->Event_ID) << " " << std::to_string(eventlog->Time * 10) << "ms ";
		// MYLOG<<eventlog->Event_Type<<eventlog->Event_ID<<eventlog->Time;
		std::string partlog;
		switch (eventlog->Event_Type)
		{
		case EVENT_TYPE_SYS_BOOT:
		{
			if (eventlog->Event_ID == EVENT_ID_BOOT_PORRST)
				partlog += ("EVENT_ID_BOOT_PORRST "); // 上电启动
			if (eventlog->Event_ID == EVENT_ID_BOOT_SOFTRST)
				partlog += ("EVENT_ID_BOOT_SOFTRST "); // 软件复位启动
			if (eventlog->Event_ID == EVENT_ID_BOOT_WDOGRST)
				partlog += ("EVENT_ID_BOOT_WDOGRST "); // 开门狗启动
			if (eventlog->Event_ID == EVENT_ID_BOOT_HARDRST)
				partlog += ("EVENT_ID_BOOT_HARDRST "); // 硬件复位启动
			break;
		}
		case EVENT_TYPE_POWER:
		{
			if (eventlog->Event_ID == EVENT_ID_POWER_LOW)
				partlog += ("EVENT_ID_POWER_LOW "); // 低压事件
			if (eventlog->Event_ID == EVENT_ID_POWER_HIGH)
				partlog += ("EVENT_ID_POWER_HIGH "); // 高压事件

			break;
		}
		case EVENT_TYPE_NET:
		{
			if (eventlog->Event_ID == EVENT_ID_NET_LINK_UP)
				partlog += ("EVENT_ID_NET_LINK_UP "); // 网络掉线
			if (eventlog->Event_ID == EVENT_ID_NET_LINK_DOWN)
				partlog += ("EVENT_ID_NET_LINK_DOWN "); // 网络重连
			if (eventlog->Event_ID == EVENT_ID_NET_SEND_DROP)
				partlog += ("EVENT_ID_NET_SEND_DROP "); // 网络发送失败
			if (eventlog->Event_ID == EVENT_ID_NET_STACK_ERR)
				partlog += ("EVENT_ID_NET_STACK_ERR "); // 网络协议栈错误，会导致系统复位

			break;
		}
		case EVENT_TYPE_TDC_DATA:
		{
			if (eventlog->Event_ID == EVENT_ID_TDC_IR_DATA_STOP)
				partlog += ("EVENT_ID_TDC_IR_DATA_STOP "); // 无红外数据
			if (eventlog->Event_ID == EVENT_ID_TDC_IR_DATA_ERROR)
				partlog += ("EVENT_ID_TDC_IR_DATA_ERROR "); // 红外数据误码过多
			if (eventlog->Event_ID == EVENT_ID_TDC_APD_HIGHTMP)
				partlog += ("EVENT_ID_TDC_APD_HIGHTMP "); // 机头apd高温

			break;
		}
		case EVENT_TYPE_MAIN_MOTOR:
		{
			if (eventlog->Event_ID == EVENT_ID_MAIN_MOTOR_STUCK)
				partlog += ("EVENT_ID_MAIN_MOTOR_STUCK "); // 底板堵转
			if (eventlog->Event_ID == EVENT_ID_MAIN_MOTOR_SLOW)
				partlog += ("EVENT_ID_MAIN_MOTOR_SLOW "); // 底板转速太慢
			if (eventlog->Event_ID == EVENT_ID_MAIN_MOTOR_FAST)
				partlog += ("EVENT_ID_MAIN_MOTOR_FAST "); // 底板转速太快

			break;
		}
		case EVENT_TYPE_MIRROR_MOTOR:
		{
			if (eventlog->Event_ID == EVENT_ID_MIRROR_MOTOR_STUCK)
				partlog += ("EVENT_ID_MIRROR_MOTOR_STUCK "); // 转镜堵转
			if (eventlog->Event_ID == EVENT_ID_MIRROR_MOTOR_SLOW)
				partlog += ("EVENT_ID_MIRROR_MOTOR_SLOW "); // 转镜转速太慢
			if (eventlog->Event_ID == EVENT_ID_MIRROR_MOTOR_FAST)
				partlog += ("EVENT_ID_MIRROR_MOTOR_FAST "); // 转镜转速太快

			break;
		}
		case EVENT_TYPE_IMU:
		{
			if (eventlog->Event_ID == EVENT_ID_IMU_INITFAILD)
				partlog += ("EVENT_ID_IMU_INITFAILD "); // IMU芯片初始化检测异常
			if (eventlog->Event_ID == EVENT_ID_IMU_NODATA)
				partlog += ("EVENT_ID_IMU_NODATA "); // IMU数据异常（读取失败）
			if (eventlog->Event_ID == EVENT_ID_IMU_TIMESTMP_LEAP)
				partlog += ("EVENT_ID_IMU_TIMESTMP_LEAP "); // IMU时间戳跳动异常，大于2个正常间隔
			if (eventlog->Event_ID == EVENT_ID_IMU_TIMESTMP_BACKW)
				partlog += ("EVENT_ID_IMU_TIMESTMP_BACKW"); // IMU时间戳回跳
			if (eventlog->Event_ID == EVENT_ID_IMU_SEND)
				partlog += ("EVENT_ID_IMU_SEND "); // IMU网络数据包发送失败
			break;
		}
		case EVENT_TYPE_EVENT:
		{
			if (eventlog->Event_ID == EVENT_ID_EVENT_CLEARED)
				partlog += ("EVENT_ID_EVENT_CLEARED"); // 事件被上位机清空
			break;
		}
		}
		result << partlog;
	}
	return result.str();
}

double PaceCatLidarSDK::PacketToPoints(BlueSeaLidarSpherPoint bluesea, LidarCloudPointData &point, StatisticsInfo &stat_info)
{
	point.reflectivity = bluesea.reflectivity;
	int32_t theta = bluesea.theta_hi;
	theta = (theta << 12) | bluesea.theta_lo;

	double ang = (90000 - theta) * M_PI / 180000;
	double vertical_ang = ang;
	double depth = bluesea.depth / 1000.0;

	double r = depth * cos(ang);
	point.z = depth * sin(ang);

	ang = bluesea.phi * M_PI / 180000;
	point.x = cos(ang) * r;
	point.y = sin(ang) * r;

	point.tag = 0;
	point.line = 0;
	if ((r != 0.0 && r < 0.035))
	{
		point.tag += 128; // 光学罩内的点进行标记，方便后续进行脏污判断
	}

	if (point.x == 0 && point.y == 0 && point.z == 0)
		point.reflectivity = 0;

	if (bluesea.depth == 0)
		stat_info.zero_point_num++;
	if (bluesea.depth < 0.2 * 1000 && bluesea.depth > 0)
		stat_info.distance_close_num++;
	stat_info.sum_point_num++;

	return vertical_ang;
}

// int tmp2=0;
void PaceCatLidarSDK::AddPacketToList(const BlueSeaLidarEthernetPacket *packet, double &last_ang, std::vector<LidarCloudPointData> &tmp_filter, std::vector<double> &tmp_ang, RunConfig *cfg)
{
	std::vector<LidarCloudPointData> tmp;
	BlueSeaLidarSpherPoint *data = (BlueSeaLidarSpherPoint *)packet->data;
	for (int i = 0; i < packet->dot_num; i++)
	{
		LidarCloudPointData point;
		double ang = PacketToPoints(data[i], point, cfg->stat_info);
		// 将点转置
		if (cfg->mr_2.mr_enable)
		{
			point.x = point.x * cfg->mr_2.rotation[0][0] + point.y * cfg->mr_2.rotation[0][1] + point.z * cfg->mr_2.rotation[0][2] + cfg->mr_2.trans[0];
			point.y = point.x * cfg->mr_2.rotation[1][0] + point.y * cfg->mr_2.rotation[1][1] + point.z * cfg->mr_2.rotation[1][2] + cfg->mr_2.trans[1];
			point.z = point.x * cfg->mr_2.rotation[2][0] + point.y * cfg->mr_2.rotation[2][1] + point.z * cfg->mr_2.rotation[2][2] + cfg->mr_2.trans[2];
		}
		point.offset_time = packet->timestamp + i * packet->time_interval * 100.0 / (packet->dot_num - 1) - cfg->frame_firstpoint_timestamp;
		if ((cfg->sfp.sfp_enable != 1) && (cfg->pfp.pfp_enable != 1))
		{
			tmp.push_back(point);
			continue;
		}
		if (cfg->pfp.pfp_enable)
		{						// 进入条件
			if (ang > last_ang) // 后续使用单独的信号量
			{
				AlgorithmAPI::OutlierFilter(tmp_filter, cfg->sfp, tmp_ang, cfg->pfp);
				tmp.insert(tmp.end(), tmp_filter.begin(), tmp_filter.end());
				tmp_ang.clear();
				tmp_filter.clear();
				tmp_ang.push_back(ang);
				tmp_filter.push_back(point);
				last_ang = ang;
				continue;
			}
			else
			{
				tmp_ang.push_back(ang);
				tmp_filter.push_back(point);
				last_ang = ang;
				continue;
			}
		}
		if (cfg->sfp.sfp_enable)
		{
			if (ang > last_ang)
			{
				AlgorithmAPI::ShadowsFilter(tmp_filter, tmp_ang, cfg->sfp, tmp_ang);
				tmp.insert(tmp.end(), tmp_filter.begin(), tmp_filter.end());
				tmp_ang.clear();
				tmp_filter.clear();
				tmp_ang.push_back(ang);
				tmp_filter.push_back(point);
			}
			else
			{
				tmp_ang.push_back(ang);
				tmp_filter.push_back(point);
			}
			last_ang = ang;
		}
		else
		{
			tmp.push_back(point);
		}
	}
	for (unsigned int i = 0; i < tmp.size(); i++)
	{
		cfg->cloud_data.push_back(tmp.at(i));
	}
	cfg->stat_info.filter_num += tmp.size();
}

void PaceCatLidarSDK::AnomalyDetection(int ID, DeBugInfo &debuginfo, StatisticsInfo &stat_info, uint64_t timestamp)
{
	float tmp_zero_point_num_factor = 1.0 * stat_info.zero_point_num / stat_info.sum_point_num;
	if (tmp_zero_point_num_factor > ZERO_POINT_FACTOR)
	{
		debuginfo.zero_pointdata_timestamp_num++;
		if (timestamp - debuginfo.zero_pointdata_timestamp_last > debuginfo.timer)
		{
			std::string err = "too many zero point num, scale factor over " + std::to_string(ZERO_POINT_FACTOR) + "value:" + std::to_string(tmp_zero_point_num_factor) + "index:" + std::to_string(debuginfo.zero_pointdata_timestamp_num);
			WriteLogData(ID, MSG_ERROR, (char *)err.c_str(), err.size());

			uint32_t error_code = 0;
			setbit(error_code, ERR_DATA_ZERO);
			PaceCatLidarSDK::getInstance()->WriteAlarmData(ID, MSG_CRITICAL, (char *)&error_code, sizeof(int));
			debuginfo.zero_pointdata_timestamp_num = 0;
			debuginfo.zero_pointdata_timestamp_last = timestamp;
		}
	}
	else
	{
		if ((timestamp - debuginfo.zero_pointdata_timestamp_last > debuginfo.timer) && (debuginfo.zero_pointdata_timestamp_num > 0))
		{
			std::string err = "too many zero point num, scale factor over " + std::to_string(ZERO_POINT_FACTOR) + "value:" + std::to_string(tmp_zero_point_num_factor) + "index:" + std::to_string(debuginfo.zero_pointdata_timestamp_num);
			WriteLogData(ID, MSG_ERROR, (char *)err.c_str(), err.size());

			uint32_t error_code = 0;
			setbit(error_code, ERR_DATA_ZERO);
			PaceCatLidarSDK::getInstance()->WriteAlarmData(ID, MSG_CRITICAL, (char *)&error_code, sizeof(int));
			debuginfo.zero_pointdata_timestamp_num = 0;
			debuginfo.zero_pointdata_timestamp_last = timestamp;
		}
	}
	float tmp_distance_close_num_factor = 1.0 * stat_info.distance_close_num / stat_info.sum_point_num;
	if (tmp_distance_close_num_factor > DISTANCE_CLOSE_FACTOR)
	{
		debuginfo.distance_close_timestamp_num++;
		if (timestamp - debuginfo.distance_close_timestamp_last > debuginfo.timer)
		{
			std::string err = "too many  distance close num,scale factor over " + std::to_string(DISTANCE_CLOSE_FACTOR) + "value:" + std::to_string(tmp_distance_close_num_factor) + "index:" + std::to_string(debuginfo.distance_close_timestamp_num);
			WriteLogData(ID, MSG_ERROR, (char *)err.c_str(), err.size());
			debuginfo.distance_close_timestamp_num = 0;
			debuginfo.distance_close_timestamp_last = timestamp;
		}
	}
	else
	{
		if ((timestamp - debuginfo.distance_close_timestamp_last > debuginfo.timer) && (debuginfo.distance_close_timestamp_num > 0))
		{
			std::string err = "too many  distance close num,scale factor over " + std::to_string(DISTANCE_CLOSE_FACTOR) + "value:" + std::to_string(tmp_distance_close_num_factor) + "index:" + std::to_string(debuginfo.distance_close_timestamp_num);
			WriteLogData(ID, MSG_ERROR, (char *)err.c_str(), err.size());
			debuginfo.distance_close_timestamp_num = 0;
			debuginfo.distance_close_timestamp_last = timestamp;
		}
	}
	float tmp_sum_point_num_factor = 1.0 * stat_info.filter_num / stat_info.sum_point_num;
	if (tmp_sum_point_num_factor < SUM_POINT_NUM_FACTOR)
	{
		std::string err = "too less sum point num,scale factor over " + std::to_string(SUM_POINT_NUM_FACTOR) + "value:" + std::to_string(tmp_sum_point_num_factor);
		WriteLogData(ID, MSG_ERROR, (char *)err.c_str(), err.size());
	}
	stat_info.zero_point_num = 0;
	stat_info.distance_close_num = 0;
	stat_info.sum_point_num = 0;
	stat_info.filter_num = 0;
}
bool PaceCatLidarSDK::udp_talk_pack(int fd_udp, std::string lidar_ip, int lidar_port, std::string command, uint16_t mode, std::string &response, uint16_t delay, uint16_t try_time)
{
	unsigned short sn = rand();
	int id = QueryIDByIp(lidar_ip);
	for (uint16_t i = 0; i < try_time; i++)
	{
		CommunicationAPI::send_cmd_udp(fd_udp, lidar_ip.c_str(), lidar_port, mode, sn, static_cast<uint16_t>(command.size()), command.c_str());
		uint64_t plantime_ms = SystemAPI::GetTimeStamp(true) + delay;
		// printf("%d %s send cmd:%s\n",__LINE__,__FUNCTION__,command.c_str());

		std::string result = " send cmd:" + command;
		WriteLogData(id, MSG_WARM, (char *)result.c_str(), result.size());

		while (SystemAPI::GetTimeStamp(true) < plantime_ms)
		{
			fd_set fds;
			FD_ZERO(&fds);
			FD_SET(fd_udp, &fds);

			struct timeval to = {0, 100};
			int ret = select(fd_udp + 1, &fds, NULL, NULL, &to);

			if (ret < 0)
			{
				printf("select error\n");
				return false;
			}
			if (ret == 0)
			{
				continue;
			}
			if (FD_ISSET(fd_udp, &fds))
			{
				sockaddr_in addr;
				socklen_t sz = sizeof(addr);

				char buf[1024] = {0};
				int nr = recvfrom(fd_udp, buf, sizeof(buf), 0, (struct sockaddr *)&addr, &sz);
				if (nr > 0)
				{
					CmdHeader *hdr = (CmdHeader *)buf;
					if (hdr->sign != 0x484c || hdr->sn != sn)
						continue;
					uint16_t cmd = ~(hdr->cmd);
					if (cmd != mode)
						continue;
					response = std::string(buf + sizeof(CmdHeader), hdr->len);
					return true;
				}
			}
		}
	}
	return false;
}

void PaceCatLidarSDK::UDPDataThreadProc(int id, HeartInfo &heartinfo)
{
	RunConfig *cfg = GetConfig(id);
	if (cfg == nullptr)
		return;
	unsigned char recv_data_buf[4096] = {0};
	char log_buf[1024] = {0};

	// 判定传入的包数量是否不符合规范[120-200]
	if (cfg->frame_package_num <= 120 || cfg->frame_package_num >= 200)
	{
		sprintf(log_buf, "one frame package num is out of range,thread end");
		WriteLogData(cfg->ID, MSG_ERROR, log_buf, strlen(log_buf));
		return;
	}

	int fd = SystemAPI::open_socket_port(cfg->listen_port, false);
	if (fd <= 0)
	{
		std::string err = "listen port open failed";
		WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
		return;
	}
	DeBugInfo debuginfo;
	memset(&debuginfo, 0, sizeof(DeBugInfo));
	debuginfo.imu_packet_idx = -1;
	debuginfo.pointcloud_packet_idx = -1;
	debuginfo.timer = LOG_TIMER * 1000000000ULL; // 定时检测时间
	debuginfo.system_timestamp_last = SystemAPI::GetTimeStamp(true);
	double last_ang = 100;
	std::vector<LidarCloudPointData> tmp_filter;
	std::vector<double> tmp_ang;
	int continuous_times = 0;
	uint64_t frame_starttime = 0;
	sockaddr_in addr;
	socklen_t sz = sizeof(addr);

	// save one frame pointcloud data
	LidarPacketData *pointclouddata = (LidarPacketData *)malloc(sizeof(LidarPacketData) + sizeof(LidarCloudPointData) * cfg->frame_package_num * 128);
	// save one packet imudata
	LidarPacketData *imudata = (LidarPacketData *)malloc(sizeof(LidarPacketData) + sizeof(LidarImuPointData));
	struct timeval to = {0, 100};
	while (cfg->run_state != QUIT)
	{
		if (m_datathread_is_block)
			m_datathread_is_block = false;
		// 每间隔一段时间对点云数据,imu数据判定是否存在发送数据异常的情况
		uint64_t current_timestamp = SystemAPI::GetTimeStamp(true);
		if (current_timestamp - debuginfo.system_timestamp_last > debuginfo.timer / 1000000)
		{
			std::lock_guard<std::mutex> lock(heartinfo.heart_mutex);
			for (size_t i = 0; i < heartinfo.lidars.size(); i++)
			{
				if (cfg->lidar_ip == heartinfo.lidars.at(i).ip)
				{
					if (!debuginfo.pointcloud_exist)
					{
						std::string err = "not find pointcloud packet,offline!";
						WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
						heartinfo.lidars.at(i).pointcloud_exist = false;
					}
					else
					{
						heartinfo.lidars.at(i).pointcloud_exist = true;
					}
					if (!debuginfo.imu_exist)
					{
						std::string err = "not find imu packet,offline!";
						WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
						heartinfo.lidars.at(i).imu_exist = false;
					}
					else
					{
						heartinfo.lidars.at(i).imu_exist = true;
					}
					break;
				}
			}

			debuginfo.system_timestamp_last = current_timestamp;
			debuginfo.pointcloud_exist = false;
			debuginfo.imu_exist = false;
		}
		if (cfg->cache_clear)
		{
			cfg->cloud_data.clear();
			cfg->package_num_idx = 0;
			debuginfo.pointcloud_timestamp_last = 0;
			debuginfo.imu_timestamp_last = 0;
			cfg->cache_clear = false;
		}
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(fd, &fds);
		int ret = select(fd + 1, &fds, NULL, NULL, &to);
		if (ret < 0)
			break;
		else if (ret == 0)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			continue;
		}
		else if (ret > 0)
		{
			if (FD_ISSET(fd, &fds))
			{
				int dw = recvfrom(fd, (char *)&recv_data_buf, sizeof(recv_data_buf), 0, (struct sockaddr *)&addr, &sz);
				if (dw > 0 && (strcmp((char *)inet_ntoa(addr.sin_addr), cfg->lidar_ip.c_str()) == 0))
				{
					if (recv_data_buf[0] == 0 || recv_data_buf[0] == 1)
					{
						// 判定是否是数据包
						const BlueSeaLidarEthernetPacket *packet = (BlueSeaLidarEthernetPacket *)&recv_data_buf;
						int packet_size = sizeof(BlueSeaLidarEthernetPacket) + packet->dot_num * sizeof(BlueSeaLidarSpherPoint);
						if (dw != packet_size)
						{
							std::string err = "find unknown packet, length error: " + std::to_string(dw) + "  " + std::to_string(packet_size);
							WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
							continue;
						}
						if (debuginfo.pointcloud_timestamp_last)
						{
							int64_t diff = int64_t(packet->timestamp - debuginfo.pointcloud_timestamp_last);
							if (diff > POINTCLOUD_TIMESTAMP_MAX_DIFF)
							{
								std::string err = "pointcloud packet interval large:" + std::to_string(diff) + "  " + std::to_string(POINTCLOUD_TIMESTAMP_MAX_DIFF);
								WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
							}
							if (diff < 0)
							{
								std::string err = "pointcloud packet time jump back:" + std::to_string(packet->timestamp) + "  " + std::to_string(debuginfo.pointcloud_timestamp_last);
								WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
							}
						}
						debuginfo.pointcloud_timestamp_last = packet->timestamp;
						debuginfo.pointcloud_exist = true;
						if (packet->udp_cnt != (uint8_t)debuginfo.pointcloud_packet_idx && debuginfo.pointcloud_packet_idx >= 0)
						{
							std::string err = "pointcloud packet lost :last " + std::to_string((uint8_t)debuginfo.pointcloud_packet_idx) + " now: " + std::to_string(packet->udp_cnt) + "index:" + std::to_string(debuginfo.pointcloud_timestamp_drop_num);
							WriteLogData(cfg->ID, MSG_WARM, (char *)err.c_str(), err.size());
						}
						debuginfo.pointcloud_packet_idx = packet->udp_cnt + 1;
						if (packet->version == 1)
						{
							// 从数据包的tag标签判定下雨系数以及单双回波 0单回波 1双回波
							if (packet->rt_v1.tag & TAG_WITH_RAIN_DETECT)
								cfg->rain = packet->rt_v1.rain;

							if (packet->rt_v1.tag & TAG_DUAL_ECHO_MODE)
								cfg->echo_mode = 1;
							else
								cfg->echo_mode = 0;

							if (packet->rt_v1.tag & TAG_MIRROR_NOT_STABLE)
							{
								debuginfo.mirror_err_timestamp_num++;
								if (packet->timestamp - debuginfo.mirror_err_timestamp_last > debuginfo.timer)
								{
									std::string err = "mirror " + std::to_string(packet->rt_v1.mirror_rpm) + " index:" + std::to_string(debuginfo.mirror_err_timestamp_num);
									WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
									debuginfo.mirror_err_timestamp_num = 0;
									debuginfo.mirror_err_timestamp_last = packet->timestamp;

									uint32_t error_code = 0;
									setbit(error_code, ERR_MIRROR_NO_STABLE);
									PaceCatLidarSDK::getInstance()->WriteAlarmData(cfg->ID, MSG_CRITICAL, (char *)&error_code, sizeof(int));
								}
								continue;
							}
							else
							{
								if ((packet->timestamp - debuginfo.mirror_err_timestamp_last > debuginfo.timer) && (debuginfo.mirror_err_timestamp_num > 0))
								{
									std::string err = "mirror " + std::to_string(packet->rt_v1.mirror_rpm) + " index:" + std::to_string(debuginfo.mirror_err_timestamp_num);
									WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
									debuginfo.mirror_err_timestamp_num = 0;
									debuginfo.mirror_err_timestamp_last = packet->timestamp;

									uint32_t error_code = 0;
									setbit(error_code, ERR_MIRROR_NO_STABLE);
									PaceCatLidarSDK::getInstance()->WriteAlarmData(cfg->ID, MSG_CRITICAL, (char *)&error_code, sizeof(int));
								}
							}

							if (packet->rt_v1.tag & TAG_MOTOR_NOT_STABLE)
							{
								debuginfo.motor_err_timestamp_num++;
								if (packet->timestamp - debuginfo.motor_err_timestamp_last > debuginfo.timer)
								{
									std::string err = "main motor " + std::to_string(packet->rt_v1.motor_rpm_x10 / 10.0) + " index:" + std::to_string(debuginfo.motor_err_timestamp_num);
									WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());

									debuginfo.motor_err_timestamp_num = 0;
									debuginfo.motor_err_timestamp_last = packet->timestamp;

									uint32_t error_code = 0;
									setbit(error_code, ERR_MOTOR_NO_STABLE);
									PaceCatLidarSDK::getInstance()->WriteAlarmData(cfg->ID, MSG_CRITICAL, (char *)&error_code, sizeof(int));
								}
								continue;
							}
							else
							{
								if ((packet->timestamp - debuginfo.motor_err_timestamp_last > debuginfo.timer) && debuginfo.motor_err_timestamp_num > 0)
								{
									std::string err = "main motor " + std::to_string(packet->rt_v1.motor_rpm_x10 / 10.0) + " index:" + std::to_string(debuginfo.motor_err_timestamp_num);
									WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());

									debuginfo.motor_err_timestamp_num = 0;
									debuginfo.motor_err_timestamp_last = packet->timestamp;

									uint32_t error_code = 0;
									setbit(error_code, ERR_MOTOR_NO_STABLE);
									PaceCatLidarSDK::getInstance()->WriteAlarmData(cfg->ID, MSG_CRITICAL, (char *)&error_code, sizeof(int));
								}
							}
						}
						size_t count = cfg->cloud_data.size();
						if (count == 0)
						{
							cfg->frame_firstpoint_timestamp = packet->timestamp;
							if (cfg->timemode == 1)
							{
								std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
								std::chrono::nanoseconds nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch());
								frame_starttime = nanoseconds.count();
							}
							else
							{
								frame_starttime = cfg->frame_firstpoint_timestamp;
							}
						}
						AddPacketToList(packet, last_ang, tmp_filter, tmp_ang, cfg);
						count = cfg->cloud_data.size();
						cfg->package_num_idx++;
						if (cfg->package_num_idx >= cfg->frame_package_num)
						{
							LidarCloudPointData *dat2 = (LidarCloudPointData *)pointclouddata->data;
							int dirt_flag = 0;
							int point_idx = 0;
							for (size_t i = 0; i < count; i++)
							{
								if (BaseAPI::isBitSet(cfg->cloud_data.at(i).tag, 7))
									dirt_flag++;

								if (!BaseAPI::isBitSet(cfg->cloud_data.at(i).tag, 6))
								{
									dat2[point_idx] = cfg->cloud_data.at(i);
									point_idx++;
								}
							}
							pointclouddata->timestamp = frame_starttime;
							pointclouddata->length = sizeof(LidarPacketData) + sizeof(LidarCloudPointData) * point_idx;
							pointclouddata->dot_num = point_idx;
							pointclouddata->frame_cnt = cfg->frame_cnt++;
							pointclouddata->data_type = LIDARPOINTCLOUD;

							if (cfg->dfp.dfp_enable)
							{
								if (dirt_flag > point_idx * cfg->dfp.dirty_factor)
								{
									continuous_times++;
									if (continuous_times > cfg->dfp.continuous_times)
									{
										std::string err = "Perhaps there is dirt or obstruction on the optical cover ! The tag points num:" + std::to_string(dirt_flag);
										WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
									}
								}
								else
								{
									continuous_times = 0;
								}
							}
							WritePointCloud(cfg->ID, 0, pointclouddata);
							memset(pointclouddata, 0, sizeof(LidarPacketData) + sizeof(LidarCloudPointData) * cfg->frame_package_num * 128);
							cfg->cloud_data.clear();
							// data debug detect
							AnomalyDetection(cfg->ID, debuginfo, cfg->stat_info, packet->timestamp);
							cfg->package_num_idx = 0;
						}
					}
					else if (recv_data_buf[0] == 0xfa && recv_data_buf[1] == 0x88)
					{
						TransBuf *trans = (TransBuf *)recv_data_buf;
						IIM42652_FIFO_PACKET_16_ST *imu_stmp = (IIM42652_FIFO_PACKET_16_ST *)(trans->data + 1);
						if (trans->idx != (uint16_t)debuginfo.imu_packet_idx && debuginfo.imu_packet_idx >= 0)
						{
							std::string err = "imudata packet lost :last " + std::to_string((uint16_t)debuginfo.imu_packet_idx) + " now: " + std::to_string(trans->idx) + " index:" + std::to_string(debuginfo.imu_timestamp_drop_num);
							WriteLogData(cfg->ID, MSG_WARM, (char *)err.c_str(), err.size());
						}
						debuginfo.imu_packet_idx = trans->idx + 1;
						debuginfo.imu_exist = true;
						if (debuginfo.imu_timestamp_last)
						{
							int64_t diff = int64_t(imu_stmp->timestamp - debuginfo.imu_timestamp_last);
							if (fabs(diff) > IMU_TIMESTAMP_MAX_DIFF)
							{
								std::string err = "imu packet interval large:" + std::to_string(diff) + "  " + std::to_string(IMU_TIMESTAMP_MAX_DIFF);
								WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
							}
							if (diff < 0)
							{
								std::string err = "imu packet time jump back:" + std::to_string(imu_stmp->timestamp) + "  " + std::to_string(debuginfo.imu_timestamp_last);
								WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
							}
						}
						debuginfo.imu_timestamp_last = imu_stmp->timestamp;
						LidarImuPointData *dat2 = (LidarImuPointData *)imudata->data;

						uint8_t imt_type = recv_data_buf[sizeof(TransBuf) + 1];
						float AccelScale = 2.0;
						float GyroScale = 2000;
						if (imt_type == 0xd4)
						{
							IMU_DATA_PACK_D4 *imu_stmp = (IMU_DATA_PACK_D4 *)(trans->data);
							GetImuFSSEL(imu_stmp->AccelScale, imu_stmp->GyroScale, AccelScale, GyroScale);
						}

						dat2->gyro_x = imu_stmp->Gyro_X * GyroScale * 2 / 0x10000 * M_PI / 180 + cfg->imu_drift.Gyro[0];
						dat2->gyro_y = imu_stmp->Gyro_Y * GyroScale * 2 / 0x10000 * M_PI / 180 + cfg->imu_drift.Gyro[1];
						dat2->gyro_z = imu_stmp->Gyro_Z * GyroScale * 2 / 0x10000 * M_PI / 180 + cfg->imu_drift.Gyro[2];
						dat2->acc_x = (imu_stmp->Accel_X * AccelScale * 2 / 0x10000);
						dat2->acc_y = (imu_stmp->Accel_Y * AccelScale * 2 / 0x10000);
						dat2->acc_z = (imu_stmp->Accel_Z * AccelScale * 2 / 0x10000);
						dat2->linear_acceleration_x = dat2->acc_x * cfg->imu_drift.R[0][0] + dat2->acc_y * cfg->imu_drift.R[0][1] + dat2->acc_z * cfg->imu_drift.R[0][2];
						dat2->linear_acceleration_y = dat2->acc_x * cfg->imu_drift.R[1][0] + dat2->acc_y * cfg->imu_drift.R[1][1] + dat2->acc_z * cfg->imu_drift.R[1][2];
						dat2->linear_acceleration_z = dat2->acc_x * cfg->imu_drift.R[2][0] + dat2->acc_y * cfg->imu_drift.R[2][1] + dat2->acc_z * cfg->imu_drift.R[2][2];
						if (cfg->timemode == 1)
							imudata->timestamp = SystemAPI::getCurrentNanoseconds();
						else
							imudata->timestamp = imu_stmp->timestamp;

						imudata->length = sizeof(LidarPacketData) + sizeof(LidarImuPointData);
						imudata->dot_num = 1;
						imudata->frame_cnt = cfg->frame_cnt;
						imudata->data_type = LIDARIMUDATA;
						WriteImuData(cfg->ID, 0, imudata);
						memset(imudata, 0, sizeof(sizeof(LidarPacketData) + sizeof(LidarImuPointData)));
					}
					else if (recv_data_buf[0] == 0xfa && recv_data_buf[1] == 0x89) // debug
					{
					}
					else if (recv_data_buf[0] == 0x4c && recv_data_buf[1] == 0x4d && (unsigned char)recv_data_buf[2] == 0x53 && (unsigned char)recv_data_buf[3] == 0x47) // alarm
					{
					}
					else
					{
						const uint8_t *ptr = (uint8_t *)&recv_data_buf;
						printf("%d : %02x %02x %02x %02x %02x %02x %02x %02x\n",
							   dw,
							   ptr[0], ptr[1], ptr[2], ptr[3],
							   ptr[4], ptr[5], ptr[6], ptr[7]);
					}
					if (debuginfo.pointcloud_timestamp_last && debuginfo.imu_timestamp_last)
					{
						int64_t diff = int64_t(debuginfo.pointcloud_timestamp_last - debuginfo.imu_timestamp_last);
						if (fabs(diff) > POINTCLOUD_IMU_TIMESTAMP_MAX_DIFF)
						{
							debuginfo.pointcloud_imu_timestamp_large_num++;
							uint64_t new_timestamp = SystemAPI::GetTimeStamp(true);
							if (new_timestamp - debuginfo.pointcloud_imu_timestamp_large_last > debuginfo.timer / 1000000)
							{
								std::string err = "pointcloud and imu packet timestamp not sync" + std::to_string(diff) + "  " + std::to_string(POINTCLOUD_IMU_TIMESTAMP_MAX_DIFF) + " index:" + std::to_string(debuginfo.pointcloud_imu_timestamp_large_num);
								WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
								debuginfo.pointcloud_imu_timestamp_large_num = 0;
								debuginfo.pointcloud_imu_timestamp_large_last = new_timestamp;
							}
						}
						else
						{
							uint64_t new_timestamp = SystemAPI::GetTimeStamp(true);
							if ((new_timestamp - debuginfo.pointcloud_imu_timestamp_large_last > debuginfo.timer / 1000000) && (debuginfo.pointcloud_imu_timestamp_large_num > 0))
							{
								std::string err = "pointcloud and imu packet timestamp not sync" + std::to_string(diff) + "  " + std::to_string(POINTCLOUD_IMU_TIMESTAMP_MAX_DIFF) + " index:" + std::to_string(debuginfo.pointcloud_imu_timestamp_large_num);
								WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
								debuginfo.pointcloud_imu_timestamp_large_num = 0;
								debuginfo.pointcloud_imu_timestamp_large_last = new_timestamp;
							}
						}
					}
				}
			}
		}
	}
	SystemAPI::closefd(fd, true);
	free(pointclouddata);
	free(imudata);
	std::string err = "recv and parse thread  end";
	WriteLogData(cfg->ID, MSG_DEBUG, (char *)err.c_str(), err.size());
}
#ifdef __unix__
void PaceCatLidarSDK::ParseLogThreadProc(int id)
{
	RunConfig *cfg = GetConfig(id);
	if (cfg == nullptr)
		return;

	ParseContext ctx = {0};
	printf("logpath:%s \n", cfg->log_path.c_str());
	// 初始化上下文
	if (init_context(&ctx, cfg->log_path.c_str()) != 0)
	{
		std::string err = "init_context error";
		WriteLogData(cfg->ID, MSG_DEBUG, (char *)err.c_str(), err.size());
		return;
	}
	std::string err = "log file size:" + std::to_string((double)ctx.file_size / (1024 * 1024 * 1024)) + "GB";
	WriteLogData(cfg->ID, MSG_DEBUG, (char *)err.c_str(), err.size());
	ctx.running = 1;
	while (ctx.running && ctx.processed_size < ctx.file_size)
	{
		if (map_next_chunk(&ctx) != 0)
		{
			break;
		}
		const uint8_t *data = (const uint8_t *)ctx.mmap_ptr;
		size_t remaining = ctx.mmap_size - ctx.current_offset;

		// 如果是文件开头，跳过PCAP全局头
		if (ctx.processed_size == 0)
		{
			if (remaining < sizeof(pcap_file_header_t))
			{
				fprintf(stderr, "Invalid PCAP file header\n");
				break;
			}

			// pcap_file_header_t *file_header = (pcap_file_header_t *)(data + ctx.current_offset);
			ctx.current_offset += sizeof(pcap_file_header_t);
			remaining -= sizeof(pcap_file_header_t);
		}
		while (remaining >= sizeof(pcap_packet_header_t))
		{
			// 读取PCAP包头部
			pcap_packet_header_t *pcap_hdr = (pcap_packet_header_t *)(data + ctx.current_offset);
			uint32_t packet_len = pcap_hdr->incl_len;

			// 移动到包数据
			ctx.current_offset += sizeof(pcap_packet_header_t);
			remaining -= sizeof(pcap_packet_header_t);

			// 检查是否有足够数据
			if (packet_len > remaining)
			{
				// 包不完整，回退并等待下一个内存块
				if (ctx.mmap_size == MEMORY_MAP_SIZE)
					ctx.current_offset -= sizeof(pcap_packet_header_t);
				else
					ctx.current_offset = ctx.mmap_size;
				break;
			}

			// 解析以太网帧
			const uint8_t *packet_data = data + ctx.current_offset;

			// 跳过以太网头部 (14字节)
			if (packet_len < 14)
			{
				ctx.current_offset += packet_len;
				remaining -= packet_len;
				continue;
			}

			// 检查以太网类型 (0x0800 = IPv4)
			uint16_t eth_type = ntohs(*(uint16_t *)(packet_data + 12));
			if (eth_type != 0x0800)
			{
				// 非IPv4包，跳过
				ctx.current_offset += packet_len;
				remaining -= packet_len;
				continue;
			}

			// 解析IP头部
			const uint8_t *ip_header = packet_data + 14;
			size_t ip_header_len = (ip_header[0] & 0x0F) * 4; // IHL字段

			if (packet_len < 14 + ip_header_len)
			{
				ctx.current_offset += packet_len;
				remaining -= packet_len;
				continue;
			}

			// 检查协议类型 (17 = UDP)
			if (ip_header[9] != 17)
			{
				// 非UDP包，跳过
				ctx.current_offset += packet_len;
				remaining -= packet_len;
				continue;
			}

			// 解析UDP头部
			const uint8_t *udp_header = ip_header + ip_header_len;
			uint16_t udp_len = ntohs(*(uint16_t *)(udp_header + 4));

			// 获取UDP负载
			const uint8_t *payload = udp_header + 8;
			size_t payload_len = udp_len - 8;

			// 解析UDP负载
			if (payload_len >= 4)
			{
				if (payload_len == 1316)
					ctx.pointcloud_num++;
				else if (payload_len == 33)
					ctx.imu_num++;
				else if (payload_len == 112)
					ctx.heart_num++;
				else
					printf("payload_len:%lu\n", payload_len);

				if(payload_len == 1316||payload_len == 33||payload_len == 112)
					m_log_queue.try_enqueue(std::string((char *)payload, payload_len));
				std::this_thread::sleep_for(std::chrono::nanoseconds(100000000 / cfg->frame_package_num));
			}
			// 移动到下一个包
			ctx.current_offset += packet_len;
			remaining -= packet_len;
			ctx.bytes_processed += packet_len;
		}
		// 更新已处理大小
		ctx.processed_size = ctx.last_mapped_offset + ctx.current_offset;
		// printf("%s %d\n",__FUNCTION__,__LINE__);
		//  printf("ctx.processed_size:%lu %lu %lu %lu\n",ctx.processed_size,ctx.pointcloud_num,ctx.imu_num,ctx.heart_num);
	}
	// 等待统计线程结束

	err = "parse log thread end:pointcloud:" + std::to_string(ctx.pointcloud_num) + "imu:" + std::to_string(ctx.imu_num) + "heart:" + std::to_string(ctx.heart_num);
	WriteLogData(cfg->ID, MSG_DEBUG, (char *)err.c_str(), err.size());
	ctx.running = 0;
	// 清理资源
	destroy_context(&ctx);
}
void PaceCatLidarSDK::PlaybackThreadProc(int id)
{
	RunConfig *cfg = GetConfig(id);
	if (cfg == nullptr)
		return;

	uint64_t frame_starttime = 0;
	int continuous_times = 0;
	double last_ang = 100;
	std::vector<LidarCloudPointData> tmp_filter;
	std::vector<double> tmp_ang;

	cfg->sfp.sfp_enable = false;
	cfg->mr_2.mr_enable = false;
	DeBugInfo debuginfo;
	memset(&debuginfo, 0, sizeof(DeBugInfo));
	debuginfo.imu_packet_idx = -1;
	debuginfo.pointcloud_packet_idx = -1;
	debuginfo.timer = LOG_TIMER * 1000000000ULL; // 定时检测时间
	LidarPacketData *pointclouddata = (LidarPacketData *)malloc(sizeof(LidarPacketData) + sizeof(LidarCloudPointData) * cfg->frame_package_num * 128);
	// save one packet imudata
	LidarPacketData *imudata = (LidarPacketData *)malloc(sizeof(LidarPacketData) + sizeof(LidarImuPointData));
	// 主处理循环
	while (cfg->run_state != QUIT)
	{
		std::string chunk;
		bool ret = m_log_queue.try_dequeue(chunk);
		if (!ret)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			continue;
		}
		unsigned char *recv_data = (unsigned char *)chunk.c_str();
		int recv_len = chunk.size();
		if ((recv_data[0] == 0x0 || recv_data[0] == 0x1) && recv_data[1] == 0x24)
		{
			// 判定是否是数据包
			const BlueSeaLidarEthernetPacket *packet = (BlueSeaLidarEthernetPacket *)recv_data;
			if (debuginfo.pointcloud_timestamp_last)
			{
				int64_t diff = int64_t(packet->timestamp - debuginfo.pointcloud_timestamp_last);
				if (diff > POINTCLOUD_TIMESTAMP_MAX_DIFF)
				{
					std::string err = "time: " + SystemAPI::getCurrentTime() + " pointcloud packet interval large:" + std::to_string(diff) + "  " + std::to_string(POINTCLOUD_TIMESTAMP_MAX_DIFF);

					WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
				}
				if (diff < 0)
				{
					std::string err = "time: " + SystemAPI::getCurrentTime() + " pointcloud packet time jump back:" + std::to_string(packet->timestamp) + "  " + std::to_string(debuginfo.pointcloud_timestamp_last);
					WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
				}
			}
			debuginfo.pointcloud_timestamp_last = packet->timestamp;
			int packet_size = sizeof(BlueSeaLidarEthernetPacket) + packet->dot_num * sizeof(BlueSeaLidarSpherPoint);

			if (recv_len != packet_size)
			{
				std::string err = "time: " + SystemAPI::getCurrentTime() + " pointcloud packet length error: " + std::to_string(recv_len) + "  " + std::to_string(packet_size);
				WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
				break;
			}
			if (packet->udp_cnt != (uint8_t)debuginfo.pointcloud_packet_idx && debuginfo.pointcloud_packet_idx >= 0)
			{
				debuginfo.pointcloud_timestamp_drop_num++;
				std::string err = "time: " + SystemAPI::getCurrentTime() + " pointcloud packet lost :last " + std::to_string((uint8_t)debuginfo.pointcloud_packet_idx) + " now: " + std::to_string(packet->udp_cnt) + "index:" + std::to_string(debuginfo.pointcloud_timestamp_drop_num);
				WriteLogData(cfg->ID, MSG_WARM, (char *)err.c_str(), err.size());
			}
			// printf("%d %d\n",debuginfo.pointcloud_packet_idx,packet->udp_cnt);
			debuginfo.pointcloud_packet_idx = packet->udp_cnt + 1;
			if (packet->version == 1)
			{
				// 从数据包的tag标签判定下雨系数以及单双回波 0单回波 1双回波
				if (packet->rt_v1.tag & TAG_WITH_RAIN_DETECT)
					cfg->rain = packet->rt_v1.rain;

				if (packet->rt_v1.tag & TAG_DUAL_ECHO_MODE)
					cfg->echo_mode = 1;
				else
					cfg->echo_mode = 0;

				if (packet->rt_v1.tag & TAG_MIRROR_NOT_STABLE)
				{
					debuginfo.mirror_err_timestamp_num++;
					if (packet->timestamp - debuginfo.mirror_err_timestamp_last > debuginfo.timer)
					{

						std::string err = "time: " + SystemAPI::getCurrentTime() + " mirror " + std::to_string(packet->rt_v1.mirror_rpm) + " index:" + std::to_string(debuginfo.mirror_err_timestamp_num);
						WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
						debuginfo.mirror_err_timestamp_num = 0;
						debuginfo.mirror_err_timestamp_last = packet->timestamp;

						uint32_t error_code = 0;
						setbit(error_code, ERR_MIRROR_NO_STABLE);
						PaceCatLidarSDK::getInstance()->WriteAlarmData(cfg->ID, MSG_CRITICAL, (char *)&error_code, sizeof(int));
					}
					continue;
				}
				else
				{
					if ((packet->timestamp - debuginfo.mirror_err_timestamp_last > debuginfo.timer) && (debuginfo.mirror_err_timestamp_num > 0))
					{
						std::string err = "time: " + SystemAPI::getCurrentTime() + " mirror " + std::to_string(packet->rt_v1.mirror_rpm) + " index:" + std::to_string(debuginfo.mirror_err_timestamp_num);
						WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
						debuginfo.mirror_err_timestamp_num = 0;
						debuginfo.mirror_err_timestamp_last = packet->timestamp;

						uint32_t error_code = 0;
						setbit(error_code, ERR_MIRROR_NO_STABLE);
						PaceCatLidarSDK::getInstance()->WriteAlarmData(cfg->ID, MSG_CRITICAL, (char *)&error_code, sizeof(int));
					}
				}

				if (packet->rt_v1.tag & TAG_MOTOR_NOT_STABLE)
				{
					debuginfo.motor_err_timestamp_num++;
					if (packet->timestamp - debuginfo.motor_err_timestamp_last > debuginfo.timer)
					{
						std::string err = "time: " + SystemAPI::getCurrentTime() + " main motor " + std::to_string(packet->rt_v1.motor_rpm_x10 / 10.0) + " index:" + std::to_string(debuginfo.motor_err_timestamp_num);
						WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());

						debuginfo.motor_err_timestamp_num = 0;
						debuginfo.motor_err_timestamp_last = packet->timestamp;

						uint32_t error_code = 0;
						setbit(error_code, ERR_MOTOR_NO_STABLE);
						PaceCatLidarSDK::getInstance()->WriteAlarmData(cfg->ID, MSG_CRITICAL, (char *)&error_code, sizeof(int));
					}
					continue;
				}
				else
				{
					if ((packet->timestamp - debuginfo.motor_err_timestamp_last > debuginfo.timer) && debuginfo.motor_err_timestamp_num > 0)
					{
						std::string err = "time: " + SystemAPI::getCurrentTime() + " main motor " + std::to_string(packet->rt_v1.motor_rpm_x10 / 10.0) + " index:" + std::to_string(debuginfo.motor_err_timestamp_num);
						WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());

						debuginfo.motor_err_timestamp_num = 0;
						debuginfo.motor_err_timestamp_last = packet->timestamp;

						uint32_t error_code = 0;
						setbit(error_code, ERR_MOTOR_NO_STABLE);
						PaceCatLidarSDK::getInstance()->WriteAlarmData(cfg->ID, MSG_CRITICAL, (char *)&error_code, sizeof(int));
					}
				}
			}
			// printf("%s %d %d\n",__FUNCTION__,__LINE__,m_package_num);
			int count = cfg->cloud_data.size();
			if (count == 0)
			{
				cfg->frame_firstpoint_timestamp = packet->timestamp;
				frame_starttime = cfg->frame_firstpoint_timestamp;
			}
			AddPacketToList(packet, last_ang, tmp_filter, tmp_ang, cfg);
			count = cfg->cloud_data.size();
			cfg->package_num_idx++;
			if (cfg->frame_package_num < 80)
			{
				std::string err = "time: " + SystemAPI::getCurrentTime() + "too few point in one frame " + std::to_string(cfg->frame_package_num);
				WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
			}
			// if (count >= cfg->frame_package_num * 128)
			if (cfg->package_num_idx >= cfg->frame_package_num)
			{
				// LidarPacketData *dat = (LidarPacketData *)malloc(sizeof(LidarPacketData) + sizeof(LidarCloudPointData) * count);
				LidarCloudPointData *dat2 = (LidarCloudPointData *)pointclouddata->data;
				SystemAPI::GetTimeStamp(true);

				int dirt_flag = 0;
				int point_idx = 0;
				for (int i = 0; i < count; i++)
				{
					if (BaseAPI::isBitSet(cfg->cloud_data.at(i).tag, 7))
						dirt_flag++;

					if (!BaseAPI::isBitSet(cfg->cloud_data.at(i).tag, 6))
					{
						dat2[point_idx] = cfg->cloud_data.at(i);
						point_idx++;
					}
				}
				pointclouddata->timestamp = frame_starttime;
				pointclouddata->length = sizeof(LidarPacketData) + sizeof(LidarCloudPointData) * point_idx;
				pointclouddata->dot_num = point_idx;
				pointclouddata->frame_cnt = cfg->frame_cnt++;
				pointclouddata->data_type = LIDARPOINTCLOUD;

				if (cfg->dfp.dfp_enable)
				{
					if (dirt_flag > point_idx * cfg->dfp.dirty_factor)
					{
						continuous_times++;
						if (continuous_times > cfg->dfp.continuous_times)
						{
							std::string err = "time: " + SystemAPI::getCurrentTime() + "Perhaps there is dirt or obstruction on the optical cover ! The tag points num:" + std::to_string(dirt_flag);
							WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
						}
					}
					else
					{
						continuous_times = 0;
					}
				}
				WritePointCloud(cfg->ID, 0, pointclouddata);
				memset(pointclouddata, 0, sizeof(LidarPacketData) + sizeof(LidarCloudPointData) * cfg->frame_package_num * 128);
				cfg->cloud_data.clear();
				AnomalyDetection(cfg->ID, debuginfo, cfg->stat_info, packet->timestamp);
				cfg->package_num_idx = 0;
			}
		}
		else if (recv_data[0] == 0xfa && recv_data[1] == 0x88)
		{
			TransBuf *trans = (TransBuf *)recv_data;
			IIM42652_FIFO_PACKET_16_ST *imu_stmp = (IIM42652_FIFO_PACKET_16_ST *)(trans->data + 1);
			if (trans->idx != (uint16_t)debuginfo.imu_packet_idx && debuginfo.imu_packet_idx >= 0)
			{
				std::string err = "time: " + SystemAPI::getCurrentTime() + " imudata packet lost :last " + std::to_string((uint16_t)debuginfo.imu_packet_idx) + " now: " + std::to_string(trans->idx) + " index:" + std::to_string(debuginfo.imu_timestamp_drop_num);
				WriteLogData(cfg->ID, MSG_WARM, (char *)err.c_str(), err.size());
			}
			debuginfo.imu_packet_idx = trans->idx + 1;
			if (debuginfo.imu_timestamp_last)
			{
				int64_t diff = int64_t(imu_stmp->timestamp - debuginfo.imu_timestamp_last);
				// printf("2:%d\n",diff);
				if (fabs(diff) > IMU_TIMESTAMP_MAX_DIFF)
				{
					std::string err = "time: " + SystemAPI::getCurrentTime() + " imu packet interval large:" + std::to_string(diff) + "  " + std::to_string(IMU_TIMESTAMP_MAX_DIFF);
					WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
				}
				if (diff < 0)
				{
					std::string err = "time: " + SystemAPI::getCurrentTime() + " imu packet time jump back:" + std::to_string(imu_stmp->timestamp) + "  " + std::to_string(debuginfo.imu_timestamp_last);
					WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
				}
			}
			debuginfo.imu_timestamp_last = imu_stmp->timestamp;
			LidarImuPointData *dat2 = (LidarImuPointData *)imudata->data;

			dat2->gyro_x = imu_stmp->Gyro_X * 4000.0 / 0x10000 * M_PI / 180 + cfg->imu_drift.Gyro[0];
			dat2->gyro_y = imu_stmp->Gyro_Y * 4000.0 / 0x10000 * M_PI / 180 + cfg->imu_drift.Gyro[1];
			dat2->gyro_z = imu_stmp->Gyro_Z * 4000.0 / 0x10000 * M_PI / 180 + cfg->imu_drift.Gyro[2];
			dat2->acc_x = (imu_stmp->Accel_X * 4.0 / 0x10000);
			dat2->acc_y = (imu_stmp->Accel_Y * 4.0 / 0x10000);
			dat2->acc_z = (imu_stmp->Accel_Z * 4.0 / 0x10000);
			// dat2->acc_x = (imu_stmp.Accel_X * 4.0 / 0x10000) * cfg->imu_drift.K[0] + cfg->imu_drift.B[0];
			// dat2->acc_y = (imu_stmp.Accel_Y * 4.0 / 0x10000)* cfg->imu_drift.K[1] + cfg->imu_drift.B[1];
			// dat2->acc_z = (imu_stmp.Accel_Z * 4.0 / 0x10000)* cfg->imu_drift.K[2] + cfg->imu_drift.B[2];
			dat2->linear_acceleration_x = dat2->acc_x * cfg->imu_drift.R[0][0] + dat2->acc_y * cfg->imu_drift.R[0][1] + dat2->acc_z * cfg->imu_drift.R[0][2];
			dat2->linear_acceleration_y = dat2->acc_x * cfg->imu_drift.R[1][0] + dat2->acc_y * cfg->imu_drift.R[1][1] + dat2->acc_z * cfg->imu_drift.R[1][2];
			dat2->linear_acceleration_z = dat2->acc_x * cfg->imu_drift.R[2][0] + dat2->acc_y * cfg->imu_drift.R[2][1] + dat2->acc_z * cfg->imu_drift.R[2][2];
			if (cfg->timemode == 1)
				imudata->timestamp = SystemAPI::getCurrentNanoseconds();
			else
				imudata->timestamp = imu_stmp->timestamp;

			imudata->length = sizeof(LidarPacketData) + sizeof(LidarImuPointData);
			imudata->dot_num = 1;
			imudata->frame_cnt = cfg->frame_cnt;
			imudata->data_type = LIDARIMUDATA;
			WriteImuData(cfg->ID, 0, imudata);
			memset(imudata, 0, sizeof(sizeof(LidarPacketData) + sizeof(LidarImuPointData)));
		}
		else if (recv_data[0] == 0xfa && recv_data[1] == 0x89) // debug
		{
		}
		else if (recv_data[0] == 0x4c && recv_data[1] == 0x48 && (unsigned char)recv_data[2] == 0xbe && (unsigned char)recv_data[3] == 0xb4) // time sync
		{
		}
		else if (recv_data[0] == 0x4c && recv_data[1] == 0x48 && (unsigned char)recv_data[2] == 0xac && (unsigned char)recv_data[3] == 0xb8) // dev_param
		{
		}
		else if (recv_data[0] == 0x4c && recv_data[1] == 0x69 && (unsigned char)recv_data[2] == 0x44 && (unsigned char)recv_data[3] == 0x41) // heart
		{
		}
		else if (recv_data[0] == 0x4c && recv_data[1] == 0x48 && (unsigned char)recv_data[2] == 0xbc && (unsigned char)recv_data[3] == 0xff)
		{

			CmdHeader *cmdheader = (CmdHeader *)recv_data;
			if (cmdheader->len == sizeof(SYS_EVENT_LOG))
			{
				std::string syslog = BaseAPI::bin_to_hex_fast(recv_data + sizeof(CmdHeader), cmdheader->len, true);
				WriteLogData(cfg->ID, MSG_WARM, (char *)syslog.c_str(), syslog.size());
			}
			else
			{
				printf("cmd:%d %s\n", cmdheader->len, recv_data + sizeof(CmdHeader));
			}
		}
		else if (recv_data[0] == 0x4c && recv_data[1] == 0x4d && (unsigned char)recv_data[2] == 0x53 && (unsigned char)recv_data[3] == 0x47) // alarm
		{
			// LidarMsgHdr* hdr = (LidarMsgHdr*)(buf);
		}
		else
		{
			const uint8_t *ptr = (uint8_t *)&recv_data;
			printf("%d : %02x %02x %02x %02x %02x %02x %02x %02x\n",
				   recv_len,
				   ptr[0], ptr[1], ptr[2], ptr[3],
				   ptr[4], ptr[5], ptr[6], ptr[7]);
		}
		if (debuginfo.pointcloud_timestamp_last && debuginfo.imu_timestamp_last)
		{
			int64_t diff = int64_t(debuginfo.pointcloud_timestamp_last - debuginfo.imu_timestamp_last);
			if (fabs(diff) > POINTCLOUD_IMU_TIMESTAMP_MAX_DIFF)
			{
				debuginfo.pointcloud_imu_timestamp_large_num++;
				uint64_t new_timestamp = SystemAPI::GetTimeStamp(true);
				if (new_timestamp - debuginfo.pointcloud_imu_timestamp_large_last > debuginfo.timer / 1000000)
				{
					std::string err = "time: " + SystemAPI::getCurrentTime() + " pointcloud and imu packet timestamp not sync" + std::to_string(diff) + "  " + std::to_string(POINTCLOUD_IMU_TIMESTAMP_MAX_DIFF) + " index:" + std::to_string(debuginfo.pointcloud_imu_timestamp_large_num);
					WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
					debuginfo.pointcloud_imu_timestamp_large_num = 0;
					debuginfo.pointcloud_imu_timestamp_large_last = new_timestamp;
				}
			}
			else
			{
				uint64_t new_timestamp = SystemAPI::GetTimeStamp(true);
				if ((new_timestamp - debuginfo.pointcloud_imu_timestamp_large_last > debuginfo.timer / 1000000) && (debuginfo.pointcloud_imu_timestamp_large_num > 0))
				{
					std::string err = "time: " + SystemAPI::getCurrentTime() + " pointcloud and imu packet timestamp not sync" + std::to_string(diff) + "  " + std::to_string(POINTCLOUD_IMU_TIMESTAMP_MAX_DIFF) + " index:" + std::to_string(debuginfo.pointcloud_imu_timestamp_large_num);
					WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
					debuginfo.pointcloud_imu_timestamp_large_num = 0;
					debuginfo.pointcloud_imu_timestamp_large_last = new_timestamp;
				}
			}
		}
	}

	std::string err = "playback thread  end";
	WriteLogData(cfg->ID, MSG_DEBUG, (char *)err.c_str(), err.size());
}
#endif
int PaceCatLidarSDK::PackNetCmd(uint16_t type, uint16_t len, uint16_t sn, const void *buf, uint8_t *netbuf)
{
	CmdHeader *hdr = (CmdHeader *)netbuf;

	hdr->sign = PACK_PREAMLE;
	hdr->cmd = type; // S_PACK;
	hdr->sn = sn;	 // rand();

	hdr->len = len;
	int len4 = ((len + 3) >> 2) * 4;
	if (len > 0)
	{
		memcpy(netbuf + sizeof(CmdHeader), buf, len);
	}

	// int n = sizeof(CmdHeader);
	uint32_t *pcrc = (uint32_t *)(netbuf + sizeof(CmdHeader) + len4);
	pcrc[0] = BaseAPI::stm32crc((uint32_t *)(netbuf + 0), len4 / 4 + 2);

	return len4 + 12;
}

int PaceCatLidarSDK::SendNetPack(int sock, uint16_t type, uint16_t len, const void *buf, char *ip, int port)
{
	uint8_t netbuf[1024];
	int netlen = PackNetCmd(type, len, rand(), buf, netbuf);

	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = inet_addr(ip);
	addr.sin_port = htons(port);

	return sendto(sock, (char *)netbuf, netlen, 0,
				  (struct sockaddr *)&addr, sizeof(struct sockaddr));
}

bool PaceCatLidarSDK::ReadCalib(int ID, std::string lidar_ip, int port)
{
#ifdef _WIN32
	WSADATA wsda;
	WSAStartup(MAKEWORD(2, 2), &wsda);
#endif // _WIN32
	int sockfd = SystemAPI::open_socket_port();
	std::string errmsg;

	uint8_t zbuf[16] = {0};
	int nr = SendNetPack(sockfd, DRIFT_RD_PACK, 4, zbuf, (char *)lidar_ip.c_str(), port);
	if (nr <= 0)
		return false;

	fd_set readfds;
	FD_ZERO(&readfds);
	FD_SET(sockfd, &readfds);

	struct timeval tv;
	tv.tv_sec = 5;
	tv.tv_usec = 0;
	int retval = select(sockfd + 1, &readfds, NULL, NULL, &tv);
	if (retval < 0)
	{
		SystemAPI::closefd(sockfd, true);
		return false;
	}
	if (retval == 0)
	{
		SystemAPI::closefd(sockfd, true);
		return false;
	}
	char buf[1024] = {0};
	sockaddr_in addr;
	socklen_t sz = sizeof(addr);
	nr = recvfrom(sockfd, (char *)&buf, sizeof(buf), 0, (struct sockaddr *)&addr, &sz);
	SystemAPI::closefd(sockfd, true);
	int packetsize = sizeof(CmdHeader) + sizeof(DriftCalib) + sizeof(uint32_t);
	if (nr == packetsize)
	{
		const CmdHeader *hdr = (CmdHeader *)buf;
		if (hdr->sign != PACK_PREAMLE)
			return false;

		uint32_t len4 = ((hdr->len + 3) >> 2) * 4;
		const uint32_t *pcrc = (uint32_t *)((char *)buf + sizeof(CmdHeader) + len4);
		uint32_t chk = BaseAPI::stm32crc((uint32_t *)buf, len4 / 4 + 2);

		if (*pcrc != chk)
			return false;

		uint16_t type = ~(hdr->cmd);
		if (type != DRIFT_RD_PACK || hdr->len < sizeof(DriftCalib))
			return false;

		DriftCalib drift;
		memcpy(&drift, buf + sizeof(CmdHeader), sizeof(drift));
		if (drift.code != DRIFT_MAGIC)
			return false;

		int id = QueryIDByIp(lidar_ip);
		RunConfig *cfg = GetConfig(id);
		cfg->imu_drift = drift.drifts.imu;
		return true;
	}

	return false;
}
void PaceCatLidarSDK::WatchDogThreadProc(bool &isrun)
{
	while (isrun)
	{
		std::this_thread::sleep_for(std::chrono::seconds(3));

		if (m_datathread_is_block)
		{
			std::string result = "data thread is block";
			WriteLogData(0, 0, (char *)result.c_str(), result.size());
		}
		if (m_heartthread_is_block)
		{
			std::string result = "heart thread is block";
			WriteLogData(0, 0, (char *)result.c_str(), result.size());
		}
		if (m_datathread_is_block || m_heartthread_is_block)
		{
			m_heartinfo.code = -1;
			m_heartinfo.value = "thread block";
		}
		m_datathread_is_block = true;
		m_heartthread_is_block = true;
	}
}
void PaceCatLidarSDK::HeartThreadProc(HeartInfo &heartinfo)
{

#ifdef _WIN32
	WSADATA wsda;
	WSAStartup(MAKEWORD(2, 2), &wsda);
#endif
	int sock = socket(AF_INET, SOCK_DGRAM, 0);
	// int yes = 1;
	// if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (char *)&yes, sizeof(yes)) < 0)
	// {
	// 	heartinfo.value = "socket init error";
	// 	heartinfo.code = SystemAPI::getLastError();
	// 	SystemAPI::closefd(sock, true);
	// 	return;
	// }
	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(HEARTPORT);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);

	int iResult = ::bind(sock, (struct sockaddr *)&addr, sizeof(addr));
	if (iResult != 0)
	{
		heartinfo.value = "bind port failed";
		heartinfo.code = SystemAPI::getLastError();
		SystemAPI::closefd(sock, true);
		std::this_thread::sleep_for(std::chrono::seconds(1));
		return;
	}
	struct ip_mreq mreq;
	mreq.imr_multiaddr.s_addr = inet_addr("225.225.225.225");
	mreq.imr_interface.s_addr = SystemAPI::get_interface_ip(heartinfo.adapter.c_str());
	if (setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&mreq, sizeof(mreq)) < 0)
	{
		heartinfo.value = "add broadcast error";
		heartinfo.code = SystemAPI::getLastError();
		SystemAPI::closefd(sock, true);
		std::this_thread::sleep_for(std::chrono::seconds(1));
		return;
	}

	uint64_t currentTimeStamp = SystemAPI::GetTimeStamp(true);
	uint64_t tto = currentTimeStamp + HEART_INTERVAL * 1000;
	socklen_t sz = sizeof(addr);
	while (heartinfo.isrun)
	{
		if (m_heartthread_is_block)
			m_heartthread_is_block = false;

		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(sock, &fds);
		struct timeval to = {0, 100};
		int ret = select(sock + 1, &fds, NULL, NULL, &to);
		if (ret == 0)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			// continue;
		}
		if (ret > 0)
		{
			char raw[4096];
			int dw = recvfrom(sock, raw, sizeof(raw), 0, (struct sockaddr *)&addr, &sz);
			if (dw == sizeof(DevHeart))
			{
				DevHeart *devheart = (DevHeart *)raw;
				std::string sn = BaseAPI::stringfilter(devheart->dev_sn, 20);
				char tmp_ip[16] = {0};
				sprintf(tmp_ip, "%d.%d.%d.%d", devheart->ip[0], devheart->ip[1], devheart->ip[2], devheart->ip[3]);
				std::string ip = BaseAPI::stringfilter(tmp_ip, 16);
				int id = PaceCatLidarSDK::getInstance()->QueryIDByIp(ip);
				bool isexist = false;
				std::lock_guard<std::mutex> lock(heartinfo.heart_mutex);
				for (size_t i = 0; i < heartinfo.lidars.size(); i++)
				{
					if (sn == heartinfo.lidars.at(i).sn)
					{
						heartinfo.lidars.at(i).timestamp = devheart->timestamp[1] * 1000 + devheart->timestamp[0] / 1000000;
						heartinfo.lidars.at(i).motor_rpm = devheart->motor_rpm;
						heartinfo.lidars.at(i).mirror_rpm = devheart->mirror_rpm;
						heartinfo.lidars.at(i).temperature = devheart->temperature;
						heartinfo.lidars.at(i).voltage = devheart->voltage;

						isexist = true;
						heartinfo.lidars.at(i).heart_exist = true;
						if (!heartinfo.lidars.at(i).isonline)
						{
							heartinfo.lidars.at(i).isonline = true;
							std::string result = sn + " " + ip + "  online" + " timestamp:" + std::to_string(heartinfo.lidars.at(i).timestamp);
							WriteLogData(id, 0, (char *)result.c_str(), result.size());
						}
						
					}
				}
				if (!isexist)
				{
					ConnectInfo info;
					info.ip = ip;
					info.sn = sn;
					info.port = devheart->port;
					info.timestamp = (devheart->timestamp[1]) * 1000 + devheart->timestamp[0] / 1000000;
					info.temperature = devheart->temperature;
					info.motor_rpm = devheart->motor_rpm;
					info.mirror_rpm = devheart->mirror_rpm;
					info.voltage = devheart->voltage;
					info.isonline = true;
					info.heart_exist = true;
					heartinfo.lidars.push_back(info);
					std::string result = sn + " " + ip + "  online" + " timestamp:" + std::to_string(info.timestamp);
					WriteLogData(id, 0, (char *)result.c_str(), result.size());
				}

				// 查询该ip所在的ID，发送严重警告事件
				int ID = PaceCatLidarSDK::getInstance()->QueryIDByIp(ip);
				// 判定条件:温度高于85  转速为0  电压:[10,32]
				if (ID >= 0)
				{
					uint32_t error_code = 0;
					if (devheart->temperature / 10.0 > 85)
						setbit(error_code, ERR_TEMPERATURE_HIGH);
					if (devheart->motor_rpm / 10.0 == 0)
						setbit(error_code, ERR_MOTOR_ZERO);
					if (devheart->mirror_rpm == 0)
						setbit(error_code, ERR_MIRROR_ZERO);
					if (devheart->voltage / 1000.0 <= 10)
						setbit(error_code, ERR_VOLTAGE_LOW);
					if (devheart->voltage / 1000.0 >= 32)
						setbit(error_code, ERR_VOLTAGE_HIGH);
					if (error_code)
						PaceCatLidarSDK::getInstance()->WriteAlarmData(ID, MSG_CRITICAL, (char *)&error_code, sizeof(int));
				}
			}
		}
		currentTimeStamp = SystemAPI::GetTimeStamp(true);
		if (currentTimeStamp > tto)
		{
			std::lock_guard<std::mutex> lock(heartinfo.heart_mutex);
			for (size_t i = 0; i < heartinfo.lidars.size(); i++)
			{
				if (heartinfo.lidars.at(i).isonline == OFFLINE && (heartinfo.lidars.at(i).pointcloud_exist || heartinfo.lidars.at(i).imu_exist || heartinfo.lidars.at(i).heart_exist))
				{
					heartinfo.lidars.at(i).isonline = ONLINE;
					std::string result = heartinfo.lidars.at(i).sn + " " + heartinfo.lidars.at(i).ip + "  online";
					WriteLogData(0, 0, (char *)result.c_str(), result.size());
				}
				else if (heartinfo.lidars.at(i).isonline == ONLINE && !heartinfo.lidars.at(i).heart_exist && !heartinfo.lidars.at(i).pointcloud_exist && !heartinfo.lidars.at(i).imu_exist)
				{
					heartinfo.lidars.at(i).isonline = OFFLINE;
					heartinfo.lidars.at(i).motor_rpm = 0;
					heartinfo.lidars.at(i).mirror_rpm = 0;
					heartinfo.lidars.at(i).temperature = 0;
					heartinfo.lidars.at(i).voltage = 0;
					heartinfo.lidars.at(i).timestamp = 0;
					std::string result = heartinfo.lidars.at(i).sn + " " + heartinfo.lidars.at(i).ip + "  offline";
					WriteLogData(0, 0, (char *)result.c_str(), result.size());
				}
				heartinfo.lidars.at(i).heart_exist = false;
			}
			tto = currentTimeStamp + HEART_INTERVAL * 1000;
		}
	}
	heartinfo.value = "thread heart end,lidar number:" + std::to_string(heartinfo.lidars.size());
	heartinfo.code = 0;
	heartinfo.lidars.clear();
	heartinfo.isrun = false;
	SystemAPI::closefd(sock, true);
}

bool PaceCatLidarSDK::FirmwareUpgrade(int id, std::string lidarip, int lidartport, int listenport, std::string path)
{
	std::string result;
	// 对文件后缀进行校验
	std::size_t idx = path.find('.');
	if (idx == std::string::npos)
	{
		result = "upgrade file format is error";
		WriteLogData(id, MSG_ERROR, (char *)result.c_str(), result.size());
		return false;
	}
	else
	{
		std::string suffix = path.substr(idx + 1);
		if (suffix != "lhr" && suffix != "lhl")
		{
			result = "upgrade file format is error:" + suffix;
			WriteLogData(id, MSG_ERROR, (char *)result.c_str(), result.size());
			return false;
		}
	}

	FirmwareInfo fileinfo;
	FirmwareInfo Lidarinfo;

	FirmwareFile *firmwareFile = LoadFirmware(path.c_str(), fileinfo);
	result = "firmware file model:" + fileinfo.model + "mcu:" + fileinfo.mcu + "motor:" + fileinfo.motor;
	WriteLogData(id, MSG_DEBUG, (char *)result.c_str(), result.size());

	if (!firmwareFile)
	{
		result = "load bin-file failed ";
		WriteLogData(id, MSG_ERROR, (char *)result.c_str(), result.size());
		return false;
	}
	// 判定是机头固件还是底板固件
	uint8_t firmwaretype;
	if (fileinfo.model == "LDS-M300-HDR" || fileinfo.model == "LDS-M200-HDR")
	{
		result = "firmware file upgrade type  is mcu";
		WriteLogData(id, MSG_DEBUG, (char *)result.c_str(), result.size());
		firmwaretype = FirmwareType::MCU;
	}
	else if (fileinfo.model == "LDS-M300-E" || fileinfo.model == "LDS-M200-E")
	{
		result = "firmware file upgrade type  is motor";
		WriteLogData(id, MSG_DEBUG, (char *)result.c_str(), result.size());
		firmwaretype = FirmwareType::MOTOR;
	}
	else
	{
		result = "firmware file model is not true:" + fileinfo.model;
		WriteLogData(id, MSG_ERROR, (char *)result.c_str(), result.size());
		free(firmwareFile);
		return false;
	}

	int fdUdp = SystemAPI::open_socket_port(listenport, false);
	int recv_len = 0;
	char recv_buf[512] = {0};
	int try_time = TRY_TIME;
	while (try_time--)
	{
		CommunicationAPI::udp_talk_pack(fdUdp, lidarip.c_str(), lidartport, 6, "LXVERH", C_PACK, recv_len, recv_buf);
		if (recv_len)
		{
			if (getLidarVersion(recv_buf, recv_len, Lidarinfo))
			{
				result = "lidar file model:" + Lidarinfo.model + "mcu:" + Lidarinfo.mcu + "motor:" + Lidarinfo.motor;
				WriteLogData(id, MSG_DEBUG, (char *)result.c_str(), result.size());
				break;
			}
		}
	}
	if (firmwaretype == FirmwareType::MCU)
	{
		if (Lidarinfo.mcu.find(fileinfo.mcu) != std::string::npos)
		{
			result = "lidar mcu version is  same ,need not upgrade!";
			WriteLogData(id, MSG_ERROR, (char *)result.c_str(), result.size());
			free(firmwareFile);
			SystemAPI::closefd(fdUdp, true);
			return true;
		}
		// 获取固件的单双回波信息
		int firmmware_echo_mode = -1; // 0为单回波  1为双回波
		if (fileinfo.mcu.find("V211") != std::string::npos)
			firmmware_echo_mode = MCUType::DUAL_M300;
		else if (fileinfo.mcu.find("V212") != std::string::npos)
			firmmware_echo_mode = MCUType::DUAL_M200;
		else if (fileinfo.mcu.find("V101") != std::string::npos)
			firmmware_echo_mode = MCUType::SINGLE;
		else
		{
			result = "firmware file echo mode is not find:" + fileinfo.mcu;
			WriteLogData(id, MSG_ERROR, (char *)result.c_str(), result.size());
			free(firmwareFile);
			SystemAPI::closefd(fdUdp, true);
			return false;
		}
		std::string firmmware_echo_mode_str = (firmmware_echo_mode == MCUType::SINGLE ? "single echo" : "dual echo");
		result = "firmware file echo mode:" + firmmware_echo_mode_str;
		WriteLogData(id, MSG_DEBUG, (char *)result.c_str(), result.size());

		// 获取雷达的单双回波信息  0为单回波  1为双回波
		int lidar_echo_mode = -2;
		if (Lidarinfo.mcu.find("V211") != std::string::npos)
			lidar_echo_mode = MCUType::DUAL_M300;
		else if (Lidarinfo.mcu.find("V212") != std::string::npos)
			lidar_echo_mode = MCUType::DUAL_M200;
		else if (Lidarinfo.mcu.find("V101") != std::string::npos)
			lidar_echo_mode = MCUType::SINGLE;
		else
		{
			result = "lidar info echo mode is not find";
			WriteLogData(id, MSG_ERROR, (char *)result.c_str(), result.size());
			free(firmwareFile);
			SystemAPI::closefd(fdUdp, true);
			return false;
		}
		std::string lidar_echo_mode_str = (lidar_echo_mode == MCUType::SINGLE ? "single echo" : "dual echo");
		result = "lidar info echo mode:" + lidar_echo_mode_str;
		WriteLogData(id, MSG_DEBUG, (char *)result.c_str(), result.size());
		if (lidar_echo_mode == firmmware_echo_mode)
		{
			firmwareFile->sent = 0;
			RangeUpInfo ru;
			memset(&ru, 0, sizeof(ru));

			strcpy(ru.m_ip, lidarip.c_str());
			ru.m_port = lidartport;
			ru.m_sock = fdUdp;
			result = "start upgrade";
			WriteLogData(id, MSG_DEBUG, (char *)result.c_str(), result.size());
			int ret = UpgradeMCU(&ru, firmwareFile);
			// printf("ret:%d\n", ret);
			SystemAPI::closefd(ru.m_sock, true);

			free(firmwareFile);
			return (ret == 0) ? true : false;
		}
		else
		{
			result = "lidar info echo mode is not same with firmware file";
			WriteLogData(id, MSG_ERROR, (char *)result.c_str(), result.size());
			free(firmwareFile);
			SystemAPI::closefd(fdUdp, true);
			return false;
		}
	}
	else if (firmwaretype == FirmwareType::MOTOR)
	{
		if (fileinfo.model != Lidarinfo.model)
		{
			result = "lidar model is not same ,can not upgrade!";
			WriteLogData(id, MSG_ERROR, (char *)result.c_str(), result.size());
			free(firmwareFile);
			SystemAPI::closefd(fdUdp, true);
			return false;
		}
		if (fileinfo.motor == Lidarinfo.motor)
		{
			result = "lidar motor version is  same ,need not upgrade!";
			WriteLogData(id, MSG_ERROR, (char *)result.c_str(), result.size());
			free(firmwareFile);
			SystemAPI::closefd(fdUdp, true);
			return true;
		}
		int ret = UpgradeMotor(fdUdp, lidarip.c_str(), lidartport, firmwareFile->len, (char *)firmwareFile->buffer);
		free(firmwareFile);
		SystemAPI::closefd(fdUdp, true);
		return (ret == 0) ? true : false;
	}

	return false;
}

bool PaceCatLidarSDK::GetImuFSSEL(int AccelScale, int GyroScale, float &AccelScale2, float &GyroScale2)
{
	switch (AccelScale)
	{
	case SET_ACCEL_FS_SEL_16g:
	{
		AccelScale2 = 16;
		break;
	}
	case SET_ACCEL_FS_SEL_8g:
	{
		AccelScale2 = 8;
		break;
	}
	case SET_ACCEL_FS_SEL_4g:
	{
		AccelScale2 = 4;
		break;
	}
	case SET_ACCEL_FS_SEL_2g:
	{
		AccelScale2 = 2;
		break;
	}
	}
	switch (GyroScale)
	{
	case SET_GYRO_FS_SEL_2000_dps:
	{
		GyroScale2 = 2000;
		break;
	}
	case SET_GYRO_FS_SEL_1000_dps:
	{
		GyroScale2 = 1000;
		break;
	}
	case SET_GYRO_FS_SEL_500_dps:
	{
		GyroScale2 = 500;
		break;
	}
	case SET_GYRO_FS_SEL_250_dps:
	{
		GyroScale2 = 250;
		break;
	}
	case SET_GYRO_FS_SEL_125_dps:
	{
		GyroScale2 = 125;
		break;
	}
	case SET_GYRO_FS_SEL_62_5_dps:
	{
		GyroScale2 = 62.5;
		break;
	}
	case SET_GYRO_FS_SEL_31_25_dps:
	{
		GyroScale2 = 31.25;
		break;
	}
	case SET_GYRO_FS_SEL_15_625_dps:
	{
		GyroScale2 = 15.625;
		break;
	}
	}
	return true;
}