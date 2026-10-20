/**
 * @file serial_pty_bridge.cpp
 * @brief 在调用方提供的两个fd间双向转发并输出有界块级时序诊断。
 *
 * 本程序不创建PTY、也不解析Dummy/ROS协议；启动后会实际读写给定fd，可连到任何调用方传入的端点，因此不是纯值单测。
 * 时间为本机steady_clock µs；chunk seq是read批次号，不是消息producer_seq或USB/CAN帧号。观察间隔与write耗时不等于端到端设备执行时延或硬实时上界。
 */
#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <thread>

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

namespace {
/**
 * @brief 最近一个read批次：seq为方向内批次号，ingress/egress为Hostµs，bytes为本次read长度。
 * stopping期间可能尚未写完整批即结束，egress仅表示forward返回时刻，不能据此证明全部bytes已发出。
 */
struct Chunk {
  std::uint64_t seq, ingress_at_us, egress_at_us, bytes;
};

/**
 * @brief 相邻read批次的Host接收间隙；保存前后chunk号/时刻及差值µs，不提供协议级丢帧判断。
 */
struct Gap {
  std::uint64_t before_seq, after_seq, before_at_us, after_at_us, gap_us;
};

/**
 * @brief 单方向唯一writer持有的统计，join两个线程后才由main读取打印。
 * bytes/chunks统计read输入；max_gap记录最大read间隔及两端锚，max_write记录forward历时。
 * recent_chunks和long_gaps各只留最近16项，未保存完整历史或对每个wire序号建账。
 */
struct Diagnostics {
  std::uint64_t bytes = 0, chunks = 0, last_at_us = 0, max_gap_us = 0, max_write_us = 0;
  std::uint64_t max_gap_before_chunk = 0, max_gap_after_chunk = 0;
  std::uint64_t max_gap_before_at_us = 0, max_gap_after_at_us = 0;
  std::deque<Chunk> recent_chunks;
  std::deque<Gap> long_gaps;
};

/**
 * @brief 取本机steady_clock绝对epoch微秒。
 * @return 单调时钟µs，非UTC、ROS /clock或设备Source时间。
 */
std::uint64_t now_us() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

/**
 * @brief 保留原fd flags并追加O_NONBLOCK。
 * @param fd 已打开的描述符。
 * @return fcntl读取/设置都成功true，否则false；不拥有或关闭fd。
 */
bool nonblocking(int fd) {
  const int flags = fcntl(fd, F_GETFL);
  return flags >= 0 && fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

/**
 * @brief 读最多4096字节并尽力非阻塞写到另一端，更新块级诊断。
 * @param source 输入fd。
 * @param target 输出fd。
 * @param threshold_us 长间隙记录阈值µs。
 * @param observed 当前方向统计，须非空且只有该方向线程写。
 * @param stopping 两线程共享的退出标志。
 * @return 暂时EAGAIN/EINTR或完成处理true，EOF/硬读写/poll错误false。
 * @details 短写按实际offset续写，EAGAIN/EINTR用10ms POLLOUT轮询；stopping可中止未写完批次，统计仍保read字节数而不是实际输出字节总数。
 */
bool forward(int source, int target, std::uint64_t threshold_us, Diagnostics *observed,
             const std::atomic<bool> &stopping) {
  std::array<char, 4096> data{};
  const ssize_t size = read(source, data.data(), data.size());
  if (size < 0 && (errno == EAGAIN || errno == EINTR)) return true;
  if (size <= 0) return false;
  const std::uint64_t received_at = now_us();
  const std::uint64_t sequence = observed->chunks + 1U;
  if (observed->last_at_us != 0U) {
    const std::uint64_t gap = received_at - observed->last_at_us;
    if (gap >= threshold_us) {
      observed->long_gaps.push_back({sequence - 1U, sequence, observed->last_at_us,
                                      received_at, gap});
      if (observed->long_gaps.size() > 16U) observed->long_gaps.pop_front();
    }
    if (gap > observed->max_gap_us) {
      observed->max_gap_us = gap;
      observed->max_gap_before_chunk = sequence - 1U;
      observed->max_gap_after_chunk = sequence;
      observed->max_gap_before_at_us = observed->last_at_us;
      observed->max_gap_after_at_us = received_at;
    }
  }
  observed->last_at_us = received_at;
  observed->bytes += static_cast<std::uint64_t>(size);
  observed->chunks = sequence;
  ssize_t offset = 0;
  while (offset < size && !stopping.load()) {
    const ssize_t written = write(target, data.data() + offset, static_cast<std::size_t>(size - offset));
    if (written > 0) {
      offset += written;
      continue;
    }
    if (written < 0 && errno != EAGAIN && errno != EINTR) return false;
    pollfd writable{target, POLLOUT, 0};
    if (poll(&writable, 1, 10) < 0 && errno != EINTR) return false;
  }
  const auto forwarded_at = now_us();
  observed->recent_chunks.push_back({sequence, received_at, forwarded_at,
                                     static_cast<std::uint64_t>(size)});
  if (observed->recent_chunks.size() > 16U) observed->recent_chunks.pop_front();
  observed->max_write_us = std::max(observed->max_write_us, forwarded_at - received_at);
  return true;
}

/**
 * @brief 一个方向的10ms poll/转发线程，任一硬错误停止整座桥。
 * @param source 源fd。
 * @param target 目标fd。
 * @param threshold_us 长read间隙阈值µs。
 * @param observed 本线程独占统计。
 * @param stopping 共享原子退出指针，须在join前存活。
 * @return 无；HUP/ERR/forward失败置stopping=true，不解释底层故障原因。
 */
void forward_loop(int source, int target, std::uint64_t threshold_us, Diagnostics *observed,
                  std::atomic<bool> *stopping) {
  while (!stopping->load()) {
    pollfd readable{source, POLLIN, 0};
    const int ready = poll(&readable, 1, 10);
    if (ready < 0) {
      if (errno == EINTR) continue;
      stopping->store(true);
      return;
    }
    if (readable.revents & (POLLHUP | POLLERR)) {
      stopping->store(true);
      return;
    }
    if ((readable.revents & POLLIN) &&
        !forward(source, target, threshold_us, observed, *stopping)) {
      stopping->store(true);
      return;
    }
  }
}

/**
 * @brief 向stdout写一个方向的JSON统计对象。
 * @param value 已停止writer后的统计引用。
 * @return 无；不清空统计或核stdout写成功。
 */
void print(const Diagnostics &value) {
  std::cout << "{\"bytes\":" << value.bytes << ",\"chunks\":" << value.chunks
            << ",\"last_at_us\":" << value.last_at_us << ",\"max_gap_us\":" << value.max_gap_us
            << ",\"max_write_us\":" << value.max_write_us
            << ",\"max_gap_before_chunk\":" << value.max_gap_before_chunk
            << ",\"max_gap_after_chunk\":" << value.max_gap_after_chunk
            << ",\"max_gap_before_at_us\":" << value.max_gap_before_at_us
            << ",\"max_gap_after_at_us\":" << value.max_gap_after_at_us << ",\"recent_chunks\":[";
  bool first = true;
  for (const Chunk &chunk : value.recent_chunks) {
    if (!first) std::cout << ',';
    first = false;
    std::cout << "{\"seq\":" << chunk.seq << ",\"at_us\":" << chunk.ingress_at_us
              << ",\"ingress_at_us\":" << chunk.ingress_at_us
              << ",\"egress_at_us\":" << chunk.egress_at_us
              << ",\"bytes\":" << chunk.bytes << '}';
  }
  std::cout << "],\"long_gaps\":[";
  first = true;
  for (const Gap &gap : value.long_gaps) {
    if (!first) std::cout << ',';
    first = false;
    std::cout << "{\"before_seq\":" << gap.before_seq << ",\"after_seq\":" << gap.after_seq
              << ",\"before_at_us\":" << gap.before_at_us << ",\"after_at_us\":" << gap.after_at_us
              << ",\"gap_us\":" << gap.gap_us << '}';
  }
  std::cout << "]}";
}

/**
 * @brief 用strtoull十进制解析参数并要求消费完整文本。
 * @param text 非空C字符串，开头负号直接拒绝；仍沿用strtoull对空白/加号等的语义。
 * @param value 可写uint64结果指针。
 * @return 无errno且至少消费一位并到串尾true；不在此限制fd范围或threshold正值。
 */
bool number(const char *text, std::uint64_t *value) {
  if (text[0] == '-') return false;
  char *end = nullptr;
  errno = 0;
  const unsigned long long parsed = std::strtoull(text, &end, 10);
  if (errno != 0 || text == end || *end != '\0') return false;
  *value = parsed;
  return true;
}
}  // namespace

/**
 * @brief 解析两fd与正阈值，启动两个方向线程，stdin事件触发收尾。
 * @param argc 参数个数（含程序名）。
 * @param argv 参数数组，模式/路径位置见下述说明。
 * @return 0表示所选检查通过；非零或断言/require异常表示失败。
 * argv1/2必须≤INT32_MAX且可设非阻塞，argv3为非零µs。stdout READY只说明线程已创建；stdin有数据/HUP/ERR或方向错误即停，join后输出两方向有限统计。
 * 本程序不显式close传入fd，不承诺可靠转存停止时的半批字节；参数失败2，正常结束0也不代表设备会话成功。
 */
int main(int argc, char **argv) {
  std::uint64_t first = 0, second = 0, threshold = 0;
  if (argc != 4 || !number(argv[1], &first) || !number(argv[2], &second) ||
      !number(argv[3], &threshold) || first > INT32_MAX || second > INT32_MAX ||
      threshold == 0U || !nonblocking(static_cast<int>(first)) ||
      !nonblocking(static_cast<int>(second))) return 2;
  Diagnostics host_to_peer, peer_to_host;
  std::atomic<bool> stopping{false};
  std::thread host_thread(forward_loop, static_cast<int>(first), static_cast<int>(second),
                          threshold, &host_to_peer, &stopping);
  std::thread peer_thread(forward_loop, static_cast<int>(second), static_cast<int>(first),
                          threshold, &peer_to_host, &stopping);
  std::cout << "READY\n" << std::flush;
  while (!stopping.load()) {
    pollfd control{STDIN_FILENO, POLLIN, 0};
    const int ready = poll(&control, 1, 10);
    if (ready < 0) {
      if (errno == EINTR) continue;
      stopping.store(true);
      break;
    }
    if (control.revents & (POLLIN | POLLHUP | POLLERR)) break;
  }
  stopping.store(true);
  host_thread.join();
  peer_thread.join();
  std::cout << "{\"clock\":\"steady_monotonic_us\",\"host_to_peer\":";
  print(host_to_peer);
  std::cout << ",\"peer_to_host\":";
  print(peer_to_host);
  std::cout << "}\n" << std::flush;
  return 0;
}
