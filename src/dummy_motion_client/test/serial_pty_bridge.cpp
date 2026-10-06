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
struct Chunk {
  std::uint64_t seq, ingress_at_us, egress_at_us, bytes;
};

struct Gap {
  std::uint64_t before_seq, after_seq, before_at_us, after_at_us, gap_us;
};

struct Diagnostics {
  std::uint64_t bytes = 0, chunks = 0, last_at_us = 0, max_gap_us = 0, max_write_us = 0;
  std::uint64_t max_gap_before_chunk = 0, max_gap_after_chunk = 0;
  std::uint64_t max_gap_before_at_us = 0, max_gap_after_at_us = 0;
  std::deque<Chunk> recent_chunks;
  std::deque<Gap> long_gaps;
};

std::uint64_t now_us() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool nonblocking(int fd) {
  const int flags = fcntl(fd, F_GETFL);
  return flags >= 0 && fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

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
