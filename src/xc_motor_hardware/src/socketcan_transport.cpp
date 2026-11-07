#include "xc_motor_hardware/socketcan_transport.hpp"
#include <cstring>
#include <cerrno>
#include <sstream>
#ifdef __linux__
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <vector>
#endif
namespace xc_motor_hardware {
SocketCanTransport::SocketCanTransport() = default;
SocketCanTransport::~SocketCanTransport() { close(); }
bool SocketCanTransport::open(const std::string &name, int bitrate, std::string *e) {
  if (name.empty()) { if (e) *e = "CAN 接口名不能为空"; return false; }
  if (bitrate <= 0) { if (e) *e = "CAN 波特率必须为正数"; return false; }
  close(); interface_name_=name;
#ifdef __linux__
  fd_=::socket(PF_CAN, SOCK_RAW, CAN_RAW); if(fd_<0){if(e)*e="创建 CAN socket 失败: "+std::string(std::strerror(errno));return false;}
  ifreq ifr{}; std::strncpy(ifr.ifr_name,name.c_str(),IFNAMSIZ-1); if(ioctl(fd_,SIOCGIFINDEX,&ifr)<0){if(e)*e="读取 CAN 接口索引失败: "+std::string(std::strerror(errno));close();return false;}
  sockaddr_can addr{}; addr.can_family=AF_CAN; addr.can_ifindex=ifr.ifr_ifindex; if(bind(fd_,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))<0){if(e)*e="绑定 CAN 接口失败: "+std::string(std::strerror(errno));close();return false;}
  int flags=fcntl(fd_,F_GETFL,0); if(flags<0 || fcntl(fd_,F_SETFL,flags|O_NONBLOCK)<0){if(e)*e="设置 CAN 非阻塞模式失败: "+std::string(std::strerror(errno));close();return false;} return true;
#else
  if(e)*e="SocketCAN 仅支持 Linux；当前平台没有 CAN RAW socket"; return false;
#endif
}
void SocketCanTransport::close() {
#ifdef __linux__
  if(fd_>=0){::close(fd_);fd_=-1;}
#else
  fd_=-1;
#endif
}
bool SocketCanTransport::is_open() const { return fd_>=0; }
bool SocketCanTransport::send(const CanFrame &f, std::string *e) {
#ifdef __linux__
  if(!is_open()){if(e)*e="CAN socket 未打开";return false;}
  if(!MotorProtocol::valid_standard_frame(f)){if(e)*e="发送帧必须是合法的经典 CAN 标准帧";return false;}
  can_frame x{}; x.can_id=f.id; x.can_dlc=f.dlc; std::memcpy(x.data,f.data,8); const auto n=::write(fd_,&x,sizeof(x)); if(n!=sizeof(x)){if(e)*e="发送 CAN 帧失败: "+std::string(std::strerror(errno));return false;} return true;
#else
  if(e)*e="当前平台不支持 SocketCAN"; return false;
#endif
}
bool SocketCanTransport::receive(CanFrame *f, int timeout_ms, std::string *e) {
#ifdef __linux__
  if(!f||!is_open()){if(e)*e="CAN socket 未打开";return false;} if(timeout_ms<0) timeout_ms=0; fd_set set;FD_ZERO(&set);FD_SET(fd_,&set); timeval tv{timeout_ms/1000,(timeout_ms%1000)*1000}; int rc=select(fd_+1,&set,nullptr,nullptr,&tv); if(rc<=0) return false; can_frame x{}; if(::read(fd_,&x,sizeof(x))!=sizeof(x)){if(e)*e="读取 CAN 帧失败";return false;} f->id=x.can_id & CAN_SFF_MASK; f->dlc=x.can_dlc; f->extended=(x.can_id & CAN_EFF_FLAG)!=0; std::memcpy(f->data,x.data,8); return true;
#else
  if(e)*e="当前平台不支持 SocketCAN"; return false;
#endif
}
bool SocketCanTransport::set_motor_filter(const std::vector<std::uint8_t> &ids, std::string *e) {
#ifdef __linux__
  if(fd_<0){if(e)*e="CAN socket 未打开";return false;} std::vector<can_filter> filters; filters.reserve(ids.size());
  for(auto id:ids) { if(!MotorProtocol::valid_motor_id(id)){if(e)*e="过滤器包含非法 motor_id";return false;} filters.push_back(can_filter{static_cast<canid_t>(id),CAN_SFF_MASK}); }
  if(!filters.empty() && setsockopt(fd_,SOL_CAN_RAW,CAN_RAW_FILTER,filters.data(),filters.size()*sizeof(can_filter))<0){if(e)*e="设置 CAN 过滤器失败: "+std::string(std::strerror(errno));return false;} return true;
#else
  (void)ids; if(e)*e="当前平台不支持 SocketCAN"; return false;
#endif
}
}  // namespace xc_motor_hardware
#ifdef __linux__
#include <fcntl.h>
#include <vector>
#endif


