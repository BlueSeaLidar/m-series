#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#include <set>
#include <vector>
#include<math.h>
#include <deque>
#include<iostream>
#include<string>
#include"protocol.h"


#define getbit(x,y)   ((x) >> (y)&1)
#define setbit(x,y) x|=(1<<y)         //将X的第Y位置1
#define clrbit(x,y) x&=~(1<<y)            //将X的第Y位清0
#define M_PI 3.14159265358979323846


//自定义结构体
struct CmdRecord
{
    char cmd[1024];//指令
    int len;//指令长度
    unsigned short sn;//随机码
    unsigned long ts;//时间戳
    unsigned  short sign;//返回协议
    unsigned int num;//重发次数
    int mode;
};

namespace BaseAPI {
    std::string stringfilter(char *str,int num);
	bool judgepcIPAddrIsValid(const char *pcIPAddr);
	bool mask_check(const char *mask);
	bool mac_check(const char *mac);
	bool checkAndMerge(int type, char*ip, char*mask, char*gateway, int port, char*result);
    std::string bin_to_hex_fast(const uint8_t *data, size_t length, bool uppercase);
    unsigned int stm32crc(unsigned int* ptr, unsigned int len);
    bool isBitSet(uint8_t num, int n);
}

namespace SystemAPI{
int open_socket_port(int port,bool isRepeat);
int open_socket_port();
int closefd(int __fd,bool isSocket);
int getLastError();
uint64_t GetTimeStamp(bool isTimeStamp_M);
uint64_t getCurrentNanoseconds();
std::string getCurrentTime();
in_addr_t get_interface_ip(const char *ifname);
}
namespace CommunicationAPI {
	void  send_cmd_udp(int fd_udp, const char* dev_ip, int dev_port, int cmd, int sn, int len, const void* snd_buf);
	bool udp_talk_pack(int fd_udp, const char * lidar_ip, int lidar_port, int send_len, const char * send_buf, int mode, int & recv_len, char * recv_buf, int delay=3, int delaynum=10000);
}

namespace AlgorithmAPI{

    double calculateDistance(const LidarCloudPointData& pointA, const LidarCloudPointData& pointB);
    bool checkWindowValid2(std::vector<LidarCloudPointData> &scan, size_t idx, size_t window, double max_distance);
    int OutlierFilter(std::vector<LidarCloudPointData> &scan_in, const ShadowsFilterParam &param, std::vector<double> &tmp_ang ,PointFilterParam &pfp);

    double getAngleWithViewpoint(float r1, float r2, double included_angle);
    int ShadowsFilter(std::vector<LidarCloudPointData> &scan_in,std::vector<double> &ang_in,const ShadowsFilterParam& param,std::vector<double> &tmp_ang);
    void setMatrixRotateParam(MatrixRotate mr,MatrixRotate_2 &mr_2);

}



#endif
