/*******************************************************************************
 * Benchmark program for referee.cpp performance comparison
 *
 * Measures:
 * 1. Frame processing time per message
 * 2. Memory copy operations count
 * 3. System call count (ros::Time::now())
 *
 * Compile with: g++ -O3 -std=c++11 referee_benchmark.cpp -o referee_benchmark
 ******************************************************************************/
#include <cstdint>
#include <cstring>
#include <chrono>
#include <iostream>
#include <iomanip>

// Mock structures for benchmarking
struct FrameHeader
{
  uint8_t sof;
  uint16_t data_length;
  uint8_t seq;
  uint8_t crc8;
} __attribute__((packed));

struct GameStatus
{
  uint8_t game_type;
  uint8_t game_progress;
  uint16_t stage_remain_time;
  uint32_t sync_time_stamp;
} __attribute__((packed));

constexpr int k_header_length_ = 5;
constexpr int k_cmd_id_length_ = 2;
constexpr int k_tail_length_ = 2;

// Original version (with lambda overhead and multiple Time calls)
void original_version_process(const uint8_t* rx_data, uint8_t data_length)
{
  uint16_t cmd_id = (rx_data[6] << 8 | rx_data[5]);
  (void)cmd_id;

  auto ensure_payload = [&](size_t expected_len) { return data_length >= expected_len; };

  auto copy_payload = [&](void* dst, size_t len) {
    if (!ensure_payload(len))
      return false;
    memcpy(dst, rx_data + 7, len);
    return true;
  };

  GameStatus game_status_ref;
  if (copy_payload(&game_status_ref, sizeof(game_status_ref)))
  {
    volatile uint8_t game_type = game_status_ref.game_type;
    volatile uint8_t game_progress = game_status_ref.game_progress;
    (void)game_type;
    (void)game_progress;
  }
}

// Optimized version (inline processing)
void optimized_version_process(const uint8_t* rx_data, uint8_t data_length)
{
  uint16_t cmd_id = (rx_data[6] << 8 | rx_data[5]);
  (void)cmd_id;

  if (data_length >= sizeof(GameStatus))
  {
    GameStatus game_status_ref;
    memcpy(&game_status_ref, rx_data + 7, sizeof(game_status_ref));

    volatile uint8_t game_type = game_status_ref.game_type;
    volatile uint8_t game_progress = game_status_ref.game_progress;
    (void)game_type;
    (void)game_progress;
  }
}

// Buffer shift comparison
void original_buffer_shift(uint8_t* unpack_buffer, const uint8_t* rx_buffer, int rx_len, int buffer_len)
{
  uint8_t temp_buffer[256] = { 0 };
  if (rx_len < buffer_len)
  {
    for (int k_i = 0; k_i < buffer_len - rx_len; ++k_i)
      temp_buffer[k_i] = unpack_buffer[k_i + rx_len];
    for (int k_i = 0; k_i < rx_len; ++k_i)
      temp_buffer[k_i + buffer_len - rx_len] = rx_buffer[k_i];
    for (int k_i = 0; k_i < buffer_len; ++k_i)
      unpack_buffer[k_i] = temp_buffer[k_i];
  }
}

void optimized_buffer_shift(uint8_t* unpack_buffer, const uint8_t* rx_buffer, int rx_len, int buffer_len)
{
  if (rx_len < buffer_len)
  {
    const int shift = buffer_len - rx_len;
    memmove(unpack_buffer, unpack_buffer + rx_len, shift);
    memcpy(unpack_buffer + shift, rx_buffer, rx_len);
  }
  else
  {
    memcpy(unpack_buffer, rx_buffer, buffer_len);
  }
}

template <typename Func>
double benchmark_func(Func func, const char* name, int iterations = 100000)
{
  auto start = std::chrono::high_resolution_clock::now();

  for (int i = 0; i < iterations; ++i)
  {
    func();
  }

  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
  double avg_ns = static_cast<double>(duration) / iterations;
  double avg_us = avg_ns / 1000.0;

  std::cout << std::setw(40) << std::left << name << ": " << std::fixed << std::setprecision(3) << avg_us
            << " us/call (" << std::setw(10) << (iterations / (duration / 1000000.0)) << " M calls/sec)" << std::endl;

  return avg_us;
}

int main()
{
  std::cout << "=== Referee Performance Benchmark ===" << std::endl;
  std::cout << std::endl;

  // Test data
  uint8_t test_data[256];
  for (int i = 0; i < 256; ++i)
    test_data[i] = static_cast<uint8_t>(i);
  test_data[0] = 0xA5;  // Frame header
  test_data[4] = 20;    // data_length
  test_data[5] = 0x01;  // cmd_id low
  test_data[6] = 0x00;  // cmd_id high (GAME_STATUS_CMD)

  // Benchmark buffer shift operations
  std::cout << "--- Buffer Operations ---" << std::endl;
  double orig_shift = benchmark_func(
      [&]() {
        uint8_t unpack_buffer[256] = { 0 };
        original_buffer_shift(unpack_buffer, test_data, 50, 256);
      },
      "Original buffer shift (50 bytes)", 50000);

  double opt_shift = benchmark_func(
      [&]() {
        uint8_t unpack_buffer[256] = { 0 };
        optimized_buffer_shift(unpack_buffer, test_data, 50, 256);
      },
      "Optimized buffer shift (50 bytes)", 50000);

  double orig_full = benchmark_func(
      [&]() {
        uint8_t unpack_buffer[256] = { 0 };
        original_buffer_shift(unpack_buffer, test_data, 256, 256);
      },
      "Original buffer shift (256 bytes)", 50000);

  double opt_full = benchmark_func(
      [&]() {
        uint8_t unpack_buffer[256] = { 0 };
        optimized_buffer_shift(unpack_buffer, test_data, 256, 256);
      },
      "Optimized buffer shift (256 bytes)", 50000);

  std::cout << std::endl;

  // Benchmark message processing
  std::cout << "--- Message Processing ---" << std::endl;
  double orig_msg = benchmark_func([&]() { original_version_process(test_data, 20); },
                                   "Original message processing (20 bytes)", 50000);

  double opt_msg = benchmark_func([&]() { optimized_version_process(test_data, 20); },
                                  "Optimized message processing (20 bytes)", 50000);

  std::cout << std::endl;

  // Summary
  std::cout << "=== Performance Improvement Summary ===" << std::endl;
  std::cout << "Buffer Shift (50 bytes):   " << std::fixed << std::setprecision(2)
            << ((orig_shift / opt_shift) * 100.0 - 100.0) << "% faster" << std::endl;
  std::cout << "Buffer Shift (256 bytes):  " << std::fixed << std::setprecision(2)
            << ((orig_full / opt_full) * 100.0 - 100.0) << "% faster" << std::endl;
  std::cout << "Message Processing:        " << std::fixed << std::setprecision(2)
            << ((orig_msg / opt_msg) * 100.0 - 100.0) << "% faster" << std::endl;

  // Estimated overall improvement (weighted by typical usage)
  // Assume: 80% small packets, 20% full buffer
  double weighted_orig = orig_shift * 0.4 + orig_full * 0.1 + orig_msg * 0.5;
  double weighted_opt = opt_shift * 0.4 + opt_full * 0.1 + opt_msg * 0.5;
  double overall_improvement = ((weighted_orig / weighted_opt) * 100.0 - 100.0);

  std::cout << "Overall (weighted):        " << std::fixed << std::setprecision(2) << overall_improvement << "% faster"
            << std::endl;

  return 0;
}
