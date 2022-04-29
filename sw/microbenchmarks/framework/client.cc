/**
 * @file client.cc
 * @brief Latency/throughput benchmarking client.
 * @author Nikita Lazarev
 */
#include <unistd.h>
#include <stdint.h>
#include <inttypes.h>
#include <algorithm>
#include <cassert>
#include <cinttypes>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <vector>

#include "CLI11.hpp"
#include "config.h"
#include "defs.h"
#include "rpc_call.h"
// #include "rpc_client.h"
// #include "rpc_client_pool.h"
// #include "rpc_types.h"
#include "utils.h"
#include "rdma_qp.h"


// typedef unsigned short uint16_t
/// HW parameters
// #ifdef PLATFORM_PAC_A10
// #  ifdef NIC_PHY_NETWORK
// /// Allocate FPGA on bus_1 for the client when running on PAC_A10 with physical
// /// networking.
// //static constexpr int kFpgaBus = dagger::cfg::platform::pac_a10_fpga_bus_1;

// /// If physical networking, running on different FPGAs, so NIC is placed by
// /// 0x20000 for both client and server.
// static constexpr uint64_t kNicAddress = 0x20000;

// #  else
// /// Allocate FPGA on bus_1 for the client when running on PAC_A10 with loopback
// /// networking.
// //static constexpr int kFpgaBus = dagger::cfg::platform::pac_a10_fpga_bus_1;

// /// If loopback, running on the same FPGA, so NIC is placed by 0x00000 for
// /// client and 0x20000 for server.
// static constexpr uint64_t kNicAddress = 0x00000;

// #  endif
// #elif PLATFORM_SDP
// /// Only loopback is possible here, use skylake_fpga_bus_1 for bus and 0x00000
// /// for NIC address.
// //static constexpr int kFpgaBus = dagger::cfg::platform::skylake_fpga_bus_1;
// static constexpr uint64_t kNicAddress = 0x00000;
// #else
// /// Only loopback is possible here, so -1 for bus and 0x00000 for address.
// static constexpr int kFpgaBus = -1;
// static constexpr uint64_t kNicAddress = 0x00000;

// #endif

static constexpr int kFpgaBus = 0xaf;
static constexpr uint64_t kNicAddress = 0x20000;
/// Networking configuration.
static constexpr char* kServerIP = "10.212.62.193";
static constexpr uint16_t kPort = 3136;


static double rdtsc_in_ns() {
  uint64_t a = dagger::utils::rdtsc();
  sleep(1);
  uint64_t b = dagger::utils::rdtsc();

  return (b - a) / 1000000000.0;
}


int add_num(dagger::RDMA* rdma, dagger::IPv4 server_addr, int remote_qp_num, int p_key, uint32_t q_key, int thread_id, int op1, int op2)
{
  int qp_num = rdma->make_qp();
  if ( qp_num != -1) {
    std::cout << "Failed to make queue pair on thread " << thread_id << std::endl;
    exit(1);
  } else {
    std::cout << "Queue  Pair created on thread " << thread_id << std::endl;
  }

  if (rdma->connect_qp(qp_num, thread_id, server_addr, remote_qp_num, p_key, q_key) != 0) {
    std::cout << "Failed to open connection on thread "<< thread_id << std::endl;
    exit(1);
  } else {
    std::cout << "Connection is open on thread " << thread_id << std::endl;
  }

  int res = op1 + op2;
  rdma->add_send_queue_entry(qp_num, &res, sizeof(res));
  rdma->send(qp_num);

  return res;
}

int main(int argc, char* argv[]) {
  // Parse input.
  // CLI::App app{"Benchmark Client"};

  size_t num_of_threads = 3;
  // app.add_option("-t, --threads", num_of_threads, "number of threads")
      // ->required();

  // CLI11_PARSE(app, argc, argv);

  // Get time/freq.
  double cycles_in_ns = rdtsc_in_ns();
  std::cout << "Cycles in ns: " << cycles_in_ns << std::endl;


  //instantiate the RDMA module
  int max_qp_pool_size = num_of_threads;
  dagger::RDMA rdma(kNicAddress, num_of_threads, max_qp_pool_size);


  // Initialize the client pool.
  volatile int res = rdma.init_nic(kFpgaBus);
  if (res != 0) return res;

  // Start the underlying nic of the pool.
  res = rdma.start_nic();
  if (res != 0) return res;

  // Enable perf thread on the nic.
  res = rdma.run_perf_thread({true, true, true, true, true}, nullptr);
  if (res != 0) return res;

  uint16_t p_key = 0; 
  uint32_t q_key;
  // Run benchmarking threads.
  std::vector<std::thread> threads;
  for (size_t thread_id = 0; thread_id < num_of_threads; ++thread_id) {
    // Open connection
    dagger::IPv4 server_addr(kServerIP, kPort + thread_id);

    q_key = thread_id;

    // Run the benchmarking thread on the client rpc_client.
    int op1 = thread_id;
    int op2 = thread_id + 1;
    std::thread thr = std::thread(&add_num, &rdma, server_addr, thread_id, p_key, q_key, thread_id, op1, op2);
    threads.push_back(std::move(thr));
  }

  // Wait until all the benchmarking threads are completed.
  for (auto& thr : threads) {
    thr.join();
  }

  // Check for HW errors on the nic.
  res = rdma.check_hw_errors();
  if (res != 0)
    std::cout << "HW errors found, check error log" << std::endl;
  else
    std::cout << "No HW errors found" << std::endl;

  // Stop the nic.
  res = rdma.stop_nic();
  if (res != 0) return res;

  return 0;
}
