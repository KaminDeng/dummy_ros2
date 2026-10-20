/**
 * @file transport_stop_exec_tests.cpp
 * @brief 跨exec重启Host复用合成peer缓存，检查Stop旧终态重放隔离。
 *
 * 复用transport_component_tests.cpp的本地主机/PTY/peer夹具；不表示连接真实MCU。每轮Host使用execl建立新进程，防止依赖继承的Stop ID生成状态。
 * 本轮注释维护只静态核对，不启动该peer/PTY集成程序。
 */
// 复用现有 PTY、真实 pico peer 和 Host 行为断言；旧 restart 用例仅覆盖同进程。
#include <sys/mman.h>
#include <new>
#define main transport_component_fixture_main
#include "transport_component_tests.cpp"
#undef main

/**
 * @brief 运行三轮Stop构建缓存、拒绝旧终态、确认当前请求的集成夹具。
 *
 * host子命令模式要求argc=8并委托run_parent；顶层创建PTY和共享mmap缓存，持久peer进程内每轮再fork连接worker。
 * 错误路径杀进程组/等待子进程；正常路径逐轮消费旧通知并标Stimulus.done，避免上轮字节跨入下一轮。
 * @param argc 命令行参数个数（含程序名）。
 * @param argv 命令行字符串数组，具体位置见上述调用约定。
 * @return 0 表示本文件全部检查通过；非零定位失败检查。
 */
int main(int argc, char **argv) {
  if (argc == 8 && std::strcmp(argv[1], "host") == 0) {
    replayed_stop_host = std::strcmp(argv[6], "replay") == 0;
    // 负例仍提供与旧终态匹配的 Idle，不能靠错 operation 的 Idle 假拒绝。
    exec_expected_stop_operation = replayed_stop_host
        ? 900U : 900U + std::strtoul(argv[7], nullptr, 10);
    return run_parent(argv[2], std::atoi(argv[3]), std::atoi(argv[4]), std::atoi(argv[5]),
        true, false, false, false, false, false, false, false, false, false,
        false, false, 0U, 0U, 0U, 0U, false, false, false, false);
  }
  SerialBridge bridge;
  if (!bridge.open_pair()) return 1;
  int requests[2]{}, responses[2]{}, stops[2]{};
  if (::pipe(requests) || ::pipe(responses) || ::pipe(stops)) return 2;
  /**
   * @brief MAP_SHARED匿名区保存合成设备层Stop缓存，跨fork worker存活。
   * Host走exec不继承生成器状态，peer缓存仍保持，从而实际覆盖跨Host进程的旧operation重放。
   */
  void *cache_memory = ::mmap(nullptr, sizeof(PersistentStopCache),
      PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
  if (cache_memory == MAP_FAILED) return 10;
  persistent_stop_cache = new (cache_memory) PersistentStopCache{};
  const auto peer = ::fork();
  if (peer < 0) return 3;
  if (peer == 0) {
    ::setpgid(0, 0);
    persistent_stop_cache_test = true;
    bridge.close_child_fds();
    ::close(requests[1]); ::close(responses[0]); ::close(stops[0]);
    for (unsigned round = 0U; round < 3U; ++round) {
      // serial 握手需要两端同时 init，必须在 context init 前放行 fresh Host。
      /**
       * @brief 先通知fresh Host再初始化串行context，确保握手两端并发进入。
       * 若在通知前单边等待context初始化，测试可能死锁而无法检验Stop语义。
       */
      if (::write(responses[1], "R", 1) != 1) ::_exit(8);
      const auto worker = ::fork();
      if (worker < 0) ::_exit(11);
      if (worker == 0)
        ::_exit(run_peer(bridge.endpoint(0), requests[0], responses[1], stops[1],
                        false, false, false, false, false));
      int worker_status = 0;
      if (::waitpid(worker, &worker_status, 0) != worker || !WIFEXITED(worker_status) ||
          WEXITSTATUS(worker_status) != 0) ::_exit(12);
    }
    ::_exit(persistent_stop_cache->round == 3U ? 0 : 9);
  }
  ::setpgid(peer, peer);
  bridge.start();
  ::close(requests[0]); ::close(stops[1]);
  int result = 0;
  // 一个 peer进程/cache 持续存活：建缓存、拒绝旧终态、确认当前终态。
  // 每轮重连 serial context，缓存属于设备层，不随通信 context 重建清空。
  // 每个 Host 都 exec，绝不继承 stop_request_ids 的进程内状态。
  for (unsigned round = 0U; round < 3U && result == 0; ++round) {
    const auto host = ::fork();
    if (host < 0) { result = 5; break; }
    if (host == 0) {
      bridge.close_child_fds();
      ::close(responses[1]);
      const auto input = std::to_string(responses[0]);
      const auto output = std::to_string(requests[1]);
      const auto stop_input = std::to_string(stops[0]);
      const auto round_arg = std::to_string(round);
      /**
       * @brief 用/proc/self/exe重建Host进程；第2轮标replay，其余current。
       * execl失败以127退出，由父进程转为该轮失败，不能继续当作成功host。
       */
      ::execl("/proc/self/exe", argv[0], "host", bridge.endpoint(1).c_str(),
          input.c_str(), output.c_str(), stop_input.c_str(),
          round == 1U ? "replay" : "current", round_arg.c_str(), nullptr);
      ::_exit(127);
    }
    int status = 0;
    if (::waitpid(host, &status, 0) != host || !WIFEXITED(status) || WEXITSTATUS(status)) {
      std::fprintf(stderr, "跨exec Stop round=%u host status=%d\n", round, status);
      result = 6;
    }
    // 旧 Host 收尾可能留下 command/query 通知；它们不属于下一轮 ready。
    for (const int fd : {responses[0], stops[0]}) {
      pollfd pending{fd, POLLIN, 0};
      char discarded[4096];
      while (::poll(&pending, 1, 0) > 0 && (pending.revents & POLLIN))
        if (::read(fd, discarded, sizeof(discarded)) <= 0) break;
    }
    Stimulus done{};
    done.done = true;
    if (::write(requests[1], &done, sizeof(done)) != sizeof(done)) result = 4;
  }
  if (result != 0) ::kill(-peer, SIGTERM);
  int peer_status = 0;
  if (::waitpid(peer, &peer_status, 0) != peer || !WIFEXITED(peer_status) ||
      WEXITSTATUS(peer_status) != 0) result = 7;
  ::close(requests[1]); ::close(responses[0]); ::close(responses[1]); ::close(stops[0]);
  ::munmap(cache_memory, sizeof(PersistentStopCache));
  if (result == 0) std::puts("同peer跨exec Stop：旧回执拒绝、当前回执确认 PASS");
  return result;
}
