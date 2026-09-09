#include "go8010_ros2_driver/go8010_protocol.hpp"
#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/msg/robot_command.hpp>
#include <robot_msgs/msg/robot_state.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <mutex>
#include <thread>
#include <chrono>
#include <array>
#include <vector>
#include <cmath>
#include <atomic>

namespace go8010 {
static constexpr double PI=3.14159265358979323846;
uint16_t crc_ccitt(const uint8_t* p,size_t n){uint16_t c=0; while(n--){c^=*p++; for(int i=0;i<8;i++) c=(c&1)?(c>>1)^0x8408:c>>1;} return c;}
std::array<uint8_t,17> encode(uint8_t id,const Command& x){std::array<uint8_t,17>b{}; uint16_t h=0xEEFE; int16_t t=std::lround(x.tau*256),v=std::lround(x.dq*256/PI/2); int32_t q=std::lround(x.q*32768/PI/2); uint16_t kp=std::lround(x.kp*1280),kd=std::lround(x.kd*1280); std::memcpy(b.data(),&h,2); b[2]=(id&15)|0x10; std::memcpy(b.data()+3,&t,2); std::memcpy(b.data()+5,&v,2); std::memcpy(b.data()+7,&q,4); std::memcpy(b.data()+11,&kp,2); std::memcpy(b.data()+13,&kd,2); uint16_t c=crc_ccitt(b.data(),15); std::memcpy(b.data()+15,&c,2); return b;}
bool decode(const uint8_t*d,size_t n,uint8_t id,State&s){if(n!=16)return false; uint16_t h;std::memcpy(&h,d,2);if(h!=0xEEFE||(d[2]&15)!=id)return false;uint16_t c;std::memcpy(&c,d+14,2);if(crc_ccitt(d,14)!=c)return false;int16_t t,v;int32_t q;std::memcpy(&t,d+3,2);std::memcpy(&v,d+5,2);std::memcpy(&q,d+7,4);s.tau=t/256.0;s.dq=v*2*PI/256;s.q=q*2*PI/32768;s.temperature=(int8_t)d[11];uint16_t st;std::memcpy(&st,d+12,2);s.error=st&7;s.mode=d[2]>>4;s.online=true;return true;}
}

class Driver final: public rclcpp::Node { using Cmd=robot_msgs::msg::RobotCommand; using St=robot_msgs::msg::RobotState; public: Driver():Node("go8010_driver"){
  port_=declare_parameter("port",std::string("/dev/ttyUSB0")); baud_=declare_parameter("baudrate",4000000); timeout_ms_=declare_parameter("response_timeout_ms",2); cmd_timeout_ms_=declare_parameter("command_timeout_ms",100); cycle_hz_=declare_parameter("cycle_hz",250.0); pub_=create_publisher<St>("/robot_state",10); sub_=create_subscription<Cmd>("/robot_command",10,[this](Cmd::SharedPtr m){std::lock_guard<std::mutex>l(mu_);cmd_=*m;last_cmd_=now();}); open_port(); worker_=std::thread([this]{loop();}); }
 ~Driver(){stop_=true;if(worker_.joinable())worker_.join();if(fd_>=0)::close(fd_);}
private: std::string port_; int baud_,timeout_ms_,cmd_timeout_ms_,fd_{-1}; double cycle_hz_; std::thread worker_; std::atomic<bool>stop_{false}; std::mutex mu_; Cmd cmd_; rclcpp::Time last_cmd_{0,0,RCL_ROS_TIME}; rclcpp::Publisher<St>::SharedPtr pub_; rclcpp::Subscription<Cmd>::SharedPtr sub_; std::array<uint8_t,12> ids_{{4,5,6,1,2,3,10,11,12,7,8,9}}; std::array<go8010::State,12> state_{};
 void open_port(){fd_=::open(port_.c_str(),O_RDWR|O_NOCTTY|O_SYNC);if(fd_<0){RCLCPP_ERROR(get_logger(),"open %s: %s",port_.c_str(),strerror(errno));return;} termios t{};tcgetattr(fd_,&t);cfmakeraw(&t);speed_t sp=B115200; if(baud_==4000000) sp=static_cast<speed_t>(0x1005); else if(baud_==2000000) sp=static_cast<speed_t>(0x1004); else if(baud_==1000000) sp=B1000000; cfsetispeed(&t,sp);cfsetospeed(&t,sp);t.c_cflag|=CLOCAL|CREAD;t.c_cc[VMIN]=0;t.c_cc[VTIME]=1;tcsetattr(fd_,TCSANOW,&t);}
 bool transact(size_t i,const go8010::Command& c){if(fd_<0)return false;auto b=go8010::encode(ids_[i],c);if(::write(fd_,b.data(),b.size())!=(ssize_t)b.size())return false;std::array<uint8_t,16>r{};size_t got=0;auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(timeout_ms_);while(got<r.size()&&std::chrono::steady_clock::now()<end){ssize_t n=::read(fd_,r.data()+got,r.size()-got);if(n>0)got+=n;}return go8010::decode(r.data(),got,ids_[i],state_[i]);}
 void loop(){auto period=std::chrono::duration<double>(1.0/cycle_hz_);while(!stop_){auto st=std::chrono::steady_clock::now();Cmd c;{std::lock_guard<std::mutex>l(mu_);c=cmd_;}if((now()-last_cmd_).seconds()*1000>cmd_timeout_ms_)for(auto&m:c.motor_command)m={};for(size_t i=0;i<ids_.size();++i){go8010::Command x{};if(i<c.motor_command.size()){auto&m=c.motor_command[i];double tr=(i%3==2)?0.5:1.0; x.q=m.q*6.33*tr; x.dq=m.dq*6.33*tr; x.tau=m.tau/(6.33*tr); x.kp=m.kp/(6.33*6.33*tr*tr); x.kd=m.kd/(6.33*6.33*tr*tr);}transact(i,x);}St out;out.motor_state.resize(ids_.size());for(size_t i=0;i<ids_.size();++i){auto&s=out.motor_state[i];double tr=(i%3==2)?0.5:1.0;s.q=state_[i].q/(6.33*tr);s.dq=state_[i].dq/(6.33*tr);s.tau_est=state_[i].tau*6.33*tr;s.ddq=0;s.cur=0;}pub_->publish(out);std::this_thread::sleep_until(st+period);}}
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Driver>());rclcpp::shutdown();return 0;}
