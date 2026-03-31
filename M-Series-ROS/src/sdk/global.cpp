#include "global.h"

#ifdef _WIN32
#pragma warning(disable : 4996)
#endif

bool mac_check(const char *mac)
{
	int dots = 0;				 // 字0符 : 的个数
	char mac_temp[17 + 1] = {0}; // mac缓存
	char *token = NULL;			 // 分割字串

	if (NULL == mac || *mac == '.')
	{
		return false; // 排除输入参数为NULL, 或者一个字符为':'的字符串
	}

	if (strlen(mac) != 17) // 长度判断
	{
		return false;
	}

	strncpy(mac_temp, mac, sizeof(mac_temp)); // mac备份

	// printf("mac:<%s\n>",mac);
	// printf("mac_temp<%s\n>",mac_temp);

	token = strtok(mac_temp, ":"); // 获取第一个子字符串

	while (token != NULL)
	{
		// printf("mac:<%s>\n",token);

		if (strlen(token) != 2) // 字串个数为2
		{
			return false;
		}

		while (*token)
		{
			// printf("*token:<%c>\n",*token);

			if ('0' <= *token && *token <= '9')
			{
				;
			}
			else if ('A' <= *token && *token <= 'F')
			{
				;
			}
			else if ('a' <= *token && *token <= 'f')
			{
				;
			}
			else
			{
				return false;
			}
			token++;
		}

		token = strtok(NULL, ":");
		dots++;
	}

	if (dots != 6) // 字串的个数
	{
		// printf("dots:<%d>\n",dots);
		return false;
	}
	else
	{
		return true;
	}
}

std::string BaseAPI::stringfilter(char *str, int num)
{
	int index = 0;
	for (int i = 0; i < num; i++)
	{
		if ((str[i] >= 45 && str[i] <= 58) || (str[i] >= 65 && str[i] <= 90) || (str[i] >= 97 && str[i] <= 122) || str[i] == 32 || str[i] == '_')
		{
			index++;
		}
		else
		{
			std::string arr = str;
			arr = arr.substr(0, index);
			return arr;
		}
	}
	return "";
}
bool BaseAPI::judgepcIPAddrIsValid(const char *pcIPAddr)
{
	int iDots = 0;	  /* 字符.的个数 */
	int iSetions = 0; /* pcIPAddr 每一部分总和（0-255）*/

	if (NULL == pcIPAddr || *pcIPAddr == '.')
	{ /*排除输入参数为NULL, 或者一个字符为'.'的字符串*/
		return false;
	}

	/* 循环取每个字符进行处理 */
	while (*pcIPAddr)
	{
		if (*pcIPAddr == '.')
		{
			iDots++;
			/* 检查 pcIPAddr 是否合法 */
			if (iSetions >= 0 && iSetions <= 255)
			{
				iSetions = 0;
				pcIPAddr++;
				continue;
			}
			else
			{
				return false;
			}
		}
		else if (*pcIPAddr >= '0' && *pcIPAddr <= '9') /*判断是不是数字*/
		{
			iSetions = iSetions * 10 + (*pcIPAddr - '0'); /*求每一段总和*/
		}
		else
		{
			return false;
		}
		pcIPAddr++;
	}

	/* 判断最后一段是否有值 如：1.1.1. */
	if ((*pcIPAddr == '\0') && (*(pcIPAddr - 1) == '.'))
	{
		return false;
	}

	/* 判断最后一段是否合法 */
	if (iSetions >= 0 && iSetions <= 255)
	{
		if (iDots == 3)
		{
			return true;
		}
	}

	return false;
}
bool BaseAPI::mask_check(const char *mask)
{
	if (BaseAPI::judgepcIPAddrIsValid(mask) != false)
	{
		unsigned int b = 0, i, n[4];
		sscanf(mask, "%u.%u.%u.%u", &n[3], &n[2], &n[1], &n[0]);

		if (strcmp(mask, "0.0.0.0") == 0) //"0.0.0.0" 是合法子网掩码，但不可设置，不可用。
			return false;

		for (i = 0; i < 4; ++i) // 将子网掩码存入32位无符号整型
		{
			b += n[i] << (i * 8);
		}

		b = ~b + 1;
		if ((b & (b - 1)) == 0) // 判断是否为2^n
			return true;
	}
	return false;
}
bool BaseAPI::mac_check(const char *mac)
{
	int dots = 0;				 // 字0符 : 的个数
	char mac_temp[17 + 1] = {0}; // mac缓存
	char *token = NULL;			 // 分割字串

	if (NULL == mac || *mac == '.')
	{
		return false; // 排除输入参数为NULL, 或者一个字符为':'的字符串
	}

	if (strlen(mac) != 17) // 长度判断
	{
		return false;
	}

	strncpy(mac_temp, mac, sizeof(mac_temp)); // mac备份

	// printf("mac:<%s\n>",mac);
	// printf("mac_temp<%s\n>",mac_temp);

	token = strtok(mac_temp, ":"); // 获取第一个子字符串

	while (token != NULL)
	{
		// printf("mac:<%s>\n",token);

		if (strlen(token) != 2) // 字串个数为2
		{
			return false;
		}

		while (*token)
		{
			// printf("*token:<%c>\n",*token);

			if ('0' <= *token && *token <= '9')
			{
				;
			}
			else if ('A' <= *token && *token <= 'F')
			{
				;
			}
			else if ('a' <= *token && *token <= 'f')
			{
				;
			}
			else
			{
				return false;
			}
			token++;
		}

		token = strtok(NULL, ":");
		dots++;
	}

	if (dots != 6) // 字串的个数
	{
		// printf("dots:<%d>\n",dots);
		return false;
	}
	else
	{
		return true;
	}
}
bool BaseAPI::checkAndMerge(int type, char *ip, char *mask, char *gateway, int port, char *result)
{
	std::string s = ip;
	int a[4] = {0};
	for (int i = 0; i < 4; i++)
	{
		size_t tmp = s.find('.', 0);
		std::string str = s.substr(0, tmp);
		a[i] = atoi(str.c_str());
		s = s.substr(tmp + 1);
		if (a[0] < 10)
			return false;
	}
	if (!judgepcIPAddrIsValid(ip))
		return false;
	if (port <= 1000 || port > 65535)
		return false;
	if (type == 0)
	{
		sprintf(result, "%03d.%03d.%03d.%03d %05d", a[0], a[1], a[2], a[3], port);
		return true;
	}
	else
	{
		if (!mask_check(mask) || !judgepcIPAddrIsValid(gateway))
			return false;
		int b[4] = {0};
		s = mask;
		for (int i = 0; i < 4; i++)
		{
			size_t tmp = s.find('.', 0);
			std::string str = s.substr(0, tmp);
			b[i] = atoi(str.c_str());
			s = s.substr(tmp + 1);
			if (b[0] == 0)
				return false;
		}
		int c[4] = {0};
		s = gateway;
		for (int i = 0; i < 4; i++)
		{
			size_t tmp = s.find('.', 0);
			std::string str = s.substr(0, tmp);
			c[i] = atoi(str.c_str());
			s = s.substr(tmp + 1);
			if (c[0] == 0)
				return false;
		}

		unsigned int str1 = 0;
		unsigned int str2 = 0;
		unsigned int str3 = 0;

		// 字符串转整形
		// sscanf(ip, "%d.%d.%d.%d", &nTmpIP[0], &nTmpIP[1], &nTmpIP[2], &nTmpIP[3]);
		for (int i = 0; i < 4; i++)
		{
			str1 += (a[i] << (24 - (i * 8)) & 0xFFFFFFFF);
		}
		for (int i = 0; i < 4; i++)
		{
			str2 += (b[i] << (24 - (i * 8)) & 0xFFFFFFFF);
		}
		for (int i = 0; i < 4; i++)
		{
			str3 += (c[i] << (24 - (i * 8)) & 0xFFFFFFFF);
		}
		if ((str1 & str2) != (str2 & str3))
			return false;

		sprintf(result, "%03d.%03d.%03d.%03d %03d.%03d.%03d.%03d %03d.%03d.%03d.%03d %05d",
				a[0], a[1], a[2], a[3], b[0], b[1], b[2], b[3], c[0], c[1], c[2], c[3], port);
		return true;
	}
	return false;
}

int SystemAPI::open_socket_port(int port, bool isRepeat)
{
#ifdef _WIN32
	WSADATA wsda; //   Structure   to   store   info
	WSAStartup(MAKEWORD(2, 2), &wsda);
#endif // _WIN32
	int fd_udp = static_cast<int>(socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP));
	if (fd_udp <= 0)
	{
		return -1;
	}
	if (isRepeat)
	{
		int opt = 1;
		int len = sizeof(opt);
		setsockopt(fd_udp, SOL_SOCKET, SO_REUSEADDR, (const char *)(&opt), len);
	}

	int rcvbufsize = 1024 * 1024 * 10;
	setsockopt(fd_udp, SOL_SOCKET, SO_RCVBUF, (char *)&rcvbufsize, sizeof(rcvbufsize));

	// DWORD nTimeout = 500;
	// int ret = setsockopt(fd_udp, SOL_SOCKET, SO_RCVTIMEO, (const char*)&nTimeout, sizeof(nTimeout));
	int time_out = 1000;
	setsockopt(fd_udp, SOL_SOCKET, SO_RCVTIMEO, (char *)&time_out, sizeof(time_out));

	// open UDP port
	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	int rt = ::bind(fd_udp, (struct sockaddr *)&addr, sizeof(addr));
	if (rt != 0)
	{
		SystemAPI::closefd(fd_udp, true);
		return -1;
	}
	return fd_udp;
}

int SystemAPI::open_socket_port()
{
#ifdef _WIN32
	WSADATA wsda; //   Structure   to   store   info
	WSAStartup(MAKEWORD(2, 2), &wsda);
#endif // _WIN32
	int fd_udp = static_cast<int>(socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP));
	if (fd_udp <= 0)
	{
		return -1;
	}

	int rcvbufsize = 1024 * 1024 * 10;
	setsockopt(fd_udp, SOL_SOCKET, SO_RCVBUF, (char *)&rcvbufsize, sizeof(rcvbufsize));

	// DWORD nTimeout = 500;
	// int ret = setsockopt(fd_udp, SOL_SOCKET, SO_RCVTIMEO, (const char*)&nTimeout, sizeof(nTimeout));
	int time_out = 1000;
	setsockopt(fd_udp, SOL_SOCKET, SO_RCVTIMEO, (char *)&time_out, sizeof(time_out));
	return fd_udp;
}


int SystemAPI::closefd(int __fd, bool isSocket)
{
#ifdef _WIN32
	if (!isSocket)
		CloseHandle(reinterpret_cast<HANDLE>(static_cast<uintptr_t>(__fd)));
	else
		closesocket(__fd);
	return 0;
#elif __linux
	UNUSED(isSocket);
	shutdown(__fd, SHUT_RDWR);
	return close(__fd);
#endif
}
int SystemAPI::getLastError()
{
#ifdef _WIN32
	return GetLastError();
#elif __linux
	return errno;
#endif
}

unsigned int BaseAPI::stm32crc(unsigned int *ptr, unsigned int len)
{
	unsigned int xbit, data;
	unsigned int crc32 = 0xFFFFFFFF;
	const unsigned int polynomial = 0x04c11db7;

	for (unsigned int i = 0; i < len; i++)
	{
		xbit = 1 << 31;
		data = ptr[i];
		for (unsigned int bits = 0; bits < 32; bits++)
		{
			if (crc32 & 0x80000000)
			{
				crc32 <<= 1;
				crc32 ^= polynomial;
			}
			else
				crc32 <<= 1;

			if (data & xbit)
				crc32 ^= polynomial;

			xbit >>= 1;
		}
	}
	return crc32;
}

uint8_t check_value(const char *buf, int len)
{
	int checksum = 0;
	for (int i = 0; i < len; i++)
	{
		checksum += buf[i];
	}
	return checksum & 0xff;
}
// void GetTimeStamp(timeval *tv, TIME_ST *timest)
// {
// #ifdef _WIN32
// 	SYSTEMTIME st;
// 	GetLocalTime(&st);

// 	tv->tv_sec = (long)time(NULL);
// 	tv->tv_usec = st.wMilliseconds;
// #elif __linux
// 	gettimeofday(tv, NULL);
// #endif
// 	timest->second = tv->tv_sec;
// 	timest->nano_second = tv->tv_usec * 1000;
// }

uint64_t SystemAPI::GetTimeStamp(bool isTimeStamp_M)
{
	timeval  tv;
#ifdef _WIN32
	SYSTEMTIME st;
	GetLocalTime(&st);

	tv.tv_sec = (long)time(NULL);
	tv.tv_usec = st.wMilliseconds;
#elif __linux
	gettimeofday(&tv, NULL);
#endif
	if (isTimeStamp_M)
		return tv.tv_sec * 1000 + tv.tv_usec / 1000;
	else
		return tv.tv_sec;
}

#include <chrono>
#include <ctime>
std::string SystemAPI::getCurrentTime()
{
	auto now = std::chrono::system_clock::now();
    
    // 转换为time_t获取年月日时分秒
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&t);
    
    // 获取毫秒部分
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    char result[80] = {0};  // 增加缓冲区大小
    sprintf(result, "%04d%02d%02d_%02d%02d%02d.%03lld", 
            1900 + tm.tm_year, tm.tm_mon + 1, tm.tm_mday, 
            tm.tm_hour, tm.tm_min, tm.tm_sec,
            static_cast<long long>(milliseconds.count()));
    
    return result;
}
void CommunicationAPI::send_cmd_udp(int fd_udp, const char *dev_ip, int dev_port, int cmd, int sn, uint16_t len, const void *snd_buf)
{
	char buffer[2048];
	CmdHeader *hdr = (CmdHeader *)buffer;
	hdr->sign = 0x484c;
	hdr->cmd = cmd;
	hdr->sn = sn;

	len = ((len + 3) >> 2) * 4;
	hdr->len = len;
	memcpy(buffer + sizeof(CmdHeader), snd_buf, len);

	unsigned int *pcrc = (unsigned int *)(buffer + sizeof(CmdHeader) + len);
	pcrc[0] = BaseAPI::stm32crc((unsigned int *)(buffer + 0), len / 4 + 2);

	sockaddr_in to;
	to.sin_family = AF_INET;
	to.sin_addr.s_addr = inet_addr(dev_ip);
	to.sin_port = htons(dev_port);

	int len2 = len + sizeof(CmdHeader) + 4;

	sendto(fd_udp, buffer, len2, 0, (struct sockaddr *)&to, sizeof(struct sockaddr));
}

bool CommunicationAPI::udp_talk_pack(int fd_udp, const char *lidar_ip, int lidar_port, int send_len, const char *send_buf, int mode, int &recv_len, char *recv_buf, int delay, int delaynum)
{
	unsigned short sn = rand();
	CommunicationAPI::send_cmd_udp(fd_udp, lidar_ip, lidar_port, mode, sn, send_len, send_buf);
	time_t t0 = time(NULL);
	int ntry = 0;
	while (time(NULL) < t0 + delay && ntry < delaynum)
	{
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(fd_udp, &fds);

		struct timeval to = {1, 0};
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

		// read UDP data
		if (FD_ISSET(fd_udp, &fds))
		{
			ntry++;
			sockaddr_in addr;
			socklen_t sz = sizeof(addr);

			char buf[1024] = {0};
			int nr = recvfrom(fd_udp, buf, sizeof(buf), 0, (struct sockaddr *)&addr, &sz);
			if (nr > 0)
			{
				CmdHeader *hdr = (CmdHeader *)buf;
				if (hdr->sign != 0x484c || hdr->sn != sn)
					continue;

				recv_len = hdr->len;
				memcpy(recv_buf, buf + sizeof(CmdHeader), recv_len);
				return true;
			}
		}
	}
	return false;
}

uint64_t SystemAPI::getCurrentNanoseconds() 
{
    // 使用高精度时钟（通常是 steady_clock 或 high_resolution_clock）
    auto now = std::chrono::high_resolution_clock::now();
    
    // 转换为纳秒时间戳
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}
std::string BaseAPI::bin_to_hex_fast(const uint8_t *data, size_t length, bool uppercase)
{
	// 创建十六进制字符查找表
	static const char hex_chars_upper[] = "0123456789ABCDEF";
	static const char hex_chars_lower[] = "0123456789abcdef";

	const char *hex_chars = uppercase ? hex_chars_upper : hex_chars_lower;

	// 预分配内存（每个字节对应2个十六进制字符）
	std::string result;
	result.reserve(length * 2);

	for (size_t i = 0; i < length; ++i)
	{
		uint8_t byte = data[i];
		result += hex_chars[(byte >> 4) & 0x0F]; // 高4位
		result += hex_chars[byte & 0x0F];		 // 低4位
	}

	return result;
}
uint32_t SystemAPI::get_interface_ip(const char *ifname)
{
	if (!ifname || strlen(ifname) == 0)
	{
		return INADDR_NONE;
	}

#ifdef _WIN32
	return 0;
#else
	// Linux/Unix 实现 (保持不变)
	int fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (fd < 0)
	{
		perror("socket");
		return INADDR_NONE;
	}

	struct ifreq ifr;
	strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
	ifr.ifr_name[IFNAMSIZ - 1] = '\0';

	if (ioctl(fd, SIOCGIFADDR, &ifr) < 0)
	{
		perror("ioctl(SIOCGIFADDR)");
		close(fd);
		return INADDR_NONE;
	}

	close(fd);
	return ((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr.s_addr;
#endif
}
bool BaseAPI::isBitSet(uint8_t num, int n)
{
	return (num & (1 << n)) != 0; // 左移 n 位并与 num 按位与
}

double AlgorithmAPI::calculateDistance(const LidarCloudPointData& pointA, const LidarCloudPointData& pointB) {
    double dx = pointB.x - pointA.x;
    double dy = pointB.y - pointA.y;
    double dz = pointB.z - pointA.z;
    
    return sqrt(dx * dx + dy * dy + dz * dz);
}

bool AlgorithmAPI::checkWindowValid2(std::vector<LidarCloudPointData> &scan, size_t idx, size_t window, double max_distance)
{
 
  unsigned int num_neighbors = 0;

  for (int y = -(int)window; y < (int)window + 1 && num_neighbors <window; y++)
  {
	int j = idx + y;
    if (j < 0 || j >= static_cast<int>(scan.size())||idx == (size_t)j)
    {
      continue;
    }
    if(scan[j].x==0&&scan[j].y==0&&scan[j].z==0)
      continue;
    // delete the re in the result
    const double d = calculateDistance(scan[idx],scan[j]);
    if (d <= max_distance)
    {
      num_neighbors++;
    }
  }
  if (num_neighbors < window)
  {
    return false; // invalid
  }
  else
  {
    return true; // effective
  }
}
int AlgorithmAPI::OutlierFilter(std::vector<LidarCloudPointData> &scan_in, const ShadowsFilterParam &param, std::vector<double> &tmp_ang ,PointFilterParam &pfp)
{

	for(unsigned int idx = 0;idx < scan_in.size();idx++){
	if(scan_in[idx].x > pfp.effective){//远距离不处理
		continue;
	}
	if(!checkWindowValid2(scan_in,idx,pfp.window_num,pfp.threshold)){
		scan_in.erase(scan_in.begin()+ idx);
	}
	}
	return 1 ;
}
double AlgorithmAPI::getAngleWithViewpoint(double r1, double r2, float included_angle)
{
	return atan2(r2 * sinf(included_angle), r1 - (r2 * cosf(included_angle)));
}
int AlgorithmAPI::ShadowsFilter(std::vector<LidarCloudPointData> &scan_in, std::vector<double> &ang_in, const ShadowsFilterParam &param, std::vector<double> &tmp_ang)
{
	std::set<int> indices_to_delete;
	for (unsigned int i = 0; i < scan_in.size() - param.window - 1; i++)
	{
		double dis_i = sqrt((scan_in[i].x * scan_in[i].x) + (scan_in[i].y * scan_in[i].y) + (scan_in[i].z * scan_in[i].z));
		if ((dis_i > param.effective_distance) | (dis_i == 0))
			continue;
		for (int y = 1; y < param.window + 1; y++)
		{
			int j = i + y;
			// double dis_i = sqrt((scan_in[i].x * scan_in[i].x) + (scan_in[i].y * scan_in[i].y) + (scan_in[i].z * scan_in[i].z));
			double dis_j = sqrt((scan_in[j].x * scan_in[j].x) + (scan_in[j].y * scan_in[j].y) + (scan_in[j].z * scan_in[j].z));
			// //屏蔽某些特殊的平面(水平面)
			// double    dis = sqrt(pow(scan_in[i].x - scan_in[j].x, 2) + pow(scan_in[i].y - scan_in[j].y, 2) + pow(scan_in[i].z - scan_in[j].z, 2));
			// double    z = fabs(scan_in[i].z - scan_in[j].z);
			// double    result = sqrt(dis*dis - z*z);
			// double   ang_final = atan2(result,z);
			// ang_final = abs(ang_final * 180 /M_PI);
			// if(ang_final < 10 || ang_final > 80) continue;
			if (j < 0 || j >= (int)scan_in.size() || (int)i == j)
				continue;
			// if(fabs(dis_i- dis_j) < 0.05)
			// continue;
			double rad = getAngleWithViewpoint(
				dis_i,
				dis_j,
				static_cast<float>(tmp_ang[i] - tmp_ang[j]));

			double angle = abs(rad * 180 / M_PI);
			// std::cout << angle<< std::endl;
			if (angle > param.max_angle || angle < param.min_angle)
			{
				// std::cout << ang_in[i]-ang_in[j]<< std::endl;
				int from, to;
				// if (dis_i < dis_j)
				{
					from = i + 1;
					to = j;
				}
				// else
				// {
				// from = j - 1;
				// to = i;
				// }

				if (from > to)
				{
					int t = from;
					from = to;
					to = t;
				}
				for (int index = from; index <= to; index++)
				{
					indices_to_delete.insert(index);
				}
			}
		}
	}

	int nr = 0;
	for (std::set<int>::iterator it = indices_to_delete.begin(); it != indices_to_delete.end(); ++it)
	{
		scan_in[*it].tag += 64;
		nr++;
	}

	return nr;
}

void AlgorithmAPI::setMatrixRotateParam(MatrixRotate mr, MatrixRotate_2 &mr_2)
{
	mr_2.mr_enable = mr.mr_enable;
	if (mr.mr_enable)
	{
		mr_2.trans[0] = mr.x;
		mr_2.trans[1] = mr.y;
		mr_2.trans[2] = mr.z;

		double cos_roll = cos(static_cast<double>(mr.roll * M_PI / 180.0));
		double cos_pitch = cos(static_cast<double>(mr.pitch * M_PI / 180.0));
		double cos_yaw = cos(static_cast<double>(mr.yaw * M_PI / 180.0));
		double sin_roll = sin(static_cast<double>(mr.roll * M_PI / 180.0));
		double sin_pitch = sin(static_cast<double>(mr.pitch * M_PI / 180.0));
		double sin_yaw = sin(static_cast<double>(mr.yaw * M_PI / 180.0));

		mr_2.rotation[0][0] = cos_pitch * cos_yaw;
		mr_2.rotation[0][1] = sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw;
		mr_2.rotation[0][2] = cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw;

		mr_2.rotation[1][0] = cos_pitch * sin_yaw;
		mr_2.rotation[1][1] = sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw;
		mr_2.rotation[1][2] = cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw;

		mr_2.rotation[2][0] = -sin_pitch;
		mr_2.rotation[2][1] = sin_roll * cos_pitch;
		mr_2.rotation[2][2] = cos_roll * cos_pitch;
	}
}