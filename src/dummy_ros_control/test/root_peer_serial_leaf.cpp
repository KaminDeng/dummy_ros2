/** @file root_peer_serial_leaf.cpp
 * @brief 测试设备侧串口握手叶层；Host仍使用未wrap的正式连接/收发/Root实现。
 * @details 仅在测试设备叶层等待初帧可读，避免正式accept先于刺激执行空读；
 * 不消费INIT、不合成ACK，正式serial helper独占握手。
 * 后续帧继续走同一正式Zenoh serial实现；不证明真实串口/MCU/电气资格。 */
#include "zenoh-pico/system/link/serial.h"
#include "zenoh-pico/protocol/definitions/serial.h"
#include <cerrno>
#include <chrono>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
/** @brief 在测试PTY设备端有限等待首帧就绪后交还正式serial accept。
 * @param socket 正式Zenoh后续使用的叶层socket，失败关闭。
 * @param device 测试PTY路径，不扫描任何物理设备。
 * @param baud 本夹具明确只使用115200。
 * @return 正式Zenoh结果；超时/错误明确失败，不假装已建立业务会话。 */
extern "C" z_result_t __wrap__z_listen_serial_from_dev(_z_sys_net_socket_t* socket,char* device,uint32_t baud) {
    if(!socket || !device || baud!=115200)return _Z_ERR_GENERIC;
    socket->_fd=::open(device,O_RDWR|O_NOCTTY|O_NONBLOCK);
    if(socket->_fd<0)return _Z_ERR_GENERIC;
    termios options{};
    if(::tcgetattr(socket->_fd,&options)!=0){_z_close_serial(socket);return _Z_ERR_GENERIC;}
    ::cfmakeraw(&options);::cfsetispeed(&options,B115200);::cfsetospeed(&options,B115200);
    if(::tcsetattr(socket->_fd,TCSANOW,&options)!=0){_z_close_serial(socket);return _Z_ERR_GENERIC;}
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(4);
    while(std::chrono::steady_clock::now()<deadline) {
        pollfd ready{socket->_fd,POLLIN,0};
        const int polled=::poll(&ready,1,20);
        if(polled<0 && errno==EINTR)continue;
        if(polled<0 || (ready.revents&(POLLHUP|POLLERR|POLLNVAL)))break;
        if(!(ready.revents&POLLIN))continue;
        // 只等待首帧可读，不消费INIT也不合成ACK；正式accept helper唯一处理握手。
        const int flags=::fcntl(socket->_fd,F_GETFL,0);
        if(flags>=0 && ::fcntl(socket->_fd,F_SETFL,flags&~O_NONBLOCK)==0)return _Z_RES_OK;
        break;
    }
    _z_close_serial(socket);return _Z_ERR_GENERIC;
}
