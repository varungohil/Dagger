/**
 * @file server.cc
 * @brief Latency/throughput benchmarking server.
 * @author Nikita Lazarev
 */
#include <unistd.h>

#include <csignal>
#include <cstdlib>
#include <iostream>

// #include "CLI11.hpp"
#include "config.h"
#include "rpc_call.h"
// #include "rpc_server_callback.h"
// #include "rpc_threaded_server.h"
#include "rpc_types.h"
#include "rdma_qp.h"

/// HW parameters.
#ifdef PLATFORM_PAC_A10
#  ifdef NIC_PHY_NETWORK
/// Allocate FPGA on bus_2 for the server when running on PAC_A10 with physical
/// networking.
static constexpr int kFpgaBus = dagger::cfg::platform::pac_a10_fpga_bus_2;

/// If physical networking, running on different FPGAs, so NIC is placed by
/// 0x20000 for both client and server.
static constexpr uint64_t kNicAddress = 0x20000;

#  else
/// Allocate FPGA on bus_1 for the server when running on PAC_A10 with loopback
/// networking.
static constexpr int kFpgaBus = dagger::cfg::platform::pac_a10_fpga_bus_1;

/// If loopback, running on the same FPGA, so NIC is placed by 0x00000 for
/// client and 0x20000 for server.
static constexpr uint64_t kNicAddress = 0x20000;

#  endif
#elif PLATFORM_SDP
/// Only loopback is possible here, use skylake_fpga_bus_1 for bus and 0x20000
/// for NIC address.
static constexpr int kFpgaBus = dagger::cfg::platform::skylake_fpga_bus_1;
static constexpr uint64_t kNicAddress = 0x20000;
#else
/// Only loopback is possible here, so -1 for bus and 0x20000 for address.
static constexpr int kFpgaBus = -1;
static constexpr uint64_t kNicAddress = 0x20000;

#endif

/// Networking configuration.
static constexpr char* kClientIP = "192.168.0.1";
static constexpr uint16_t kPort = 3136;

/// Ctl-C handler.
// static volatile int keepRunning = 1;
// void intHandler(int dummy) { keepRunning = 0; }

void get_data(dagger::RDMA* rdma, dagger::IPv4 server_addr, int remote_qp_num, int p_key, uint32_t q_key, int qp_id, int* res)
{
  int qp_num = rdma->make_qp();
  if ( qp_num != -1) {
    std::cout << "Failed to make queue pair on thread " << qp_id << std::endl;
    exit(1);
  } else {
    std::cout << "Queue  Pair created on thread " << qp_id << std::endl;
  }

  if (rdma->connect_qp(qp_num, qp_id, server_addr, remote_qp_num, p_key, q_key) != 0) {
    std::cout << "Failed to open connection on thread "<< qp_id << std::endl;
    exit(1);
  } else {
    std::cout << "Connection is open on thread " << qp_id << std::endl;
  }

  rdma->add_recv_queue_entry(qp_num, &res, sizeof(res));
  rdma->recv(qp_num);
  
  // rdma->stop_recv(qp_num);
}


// <max number of threads, run duration>
int main(int argc, char* argv[]) {
  // Parse input.
  // CLI::App app{"Benchmark Server"};

  // size_t num_qps;
  // app.add_option("-q, --queuepairs", num_qps, "number of queue pairs")
      // ->required();

  // int load_balancer;
  // app.add_option("-l, --load-balancer", load_balancer, "load balancer")
  //     ->required();

  // CLI11_PARSE(app, argc, argv);

  size_t num_qps = 3;

  //instantiate the RDMA module
  int max_qp_pool_size = num_qps;
  dagger::RDMA rdma(kNicAddress, num_qps, max_qp_pool_size)

  // Initialize the server.
  int res = rdma.init_nic(kFpgaBus);
  if (res != 0) return res;

  // Start the server.
  res = rdma.start_nic();
  if (res != 0) return res;

  // Enable perf thread on the nic.
  res = rdma.run_perf_thread({true, true, true, true, true}, nullptr);
  if (res != 0) return res;

  // // Open connections.
  // for (size_t i = 0; i < num_qps; ++i) {
  //   dagger::IPv4 client_addr(kClientIP, kPort);
  //   if (server.connect(client_addr, i, i) != 0) {
  //     std::cout << "Failed to open connection on server" << std::endl;
  //     exit(1);
  //   } else {
  //     std::cout << "Connection is open on server" << std::endl;
  //   }
  // }

  // Run benchmarking threads.
  std::vector<int> results;
  results.resize(num_qps);
  for (size_t qp_id = 0; qp_id < num_qps; ++qp_id) {
    // Open connection
    dagger::IPv4 server_addr(kServerIP, kPort + qp_id);
    int qp_num = rdma.make_qp();
    if ( qp_num != -1) {
      std::cout << "Failed to make queue pair on thread " << qp_id << std::endl;
      exit(1);
    } else {
      std::cout << "Queue  Pair created on thread " << qp_id << std::endl;
    }

    if (rdma.connect_qp(qp_num, qp_id, server_addr, qp_num, p_key, q_key) != 0) {
      std::cout << "Failed to open connection on thread "<< qp_id << std::endl;
      exit(1);
    } else {
      std::cout << "Connection is open on thread " << qp_id << std::endl;
    }

    rdma->add_recv_queue_entry(qp_num, &results[qp_id], sizeof(int));
    rdma->recv(qp_num);
  }

  bool all_data_available = 0;
  
  while(all_data_available == 0){
    for (size_t qp_id = 0; qp_id < num_qps; ++qp_id)
    {
      all_data_available = all_data_avaialble && rdma.is_data_available(qp_id);
    }
  } 
  for (size_t qp_id = 0; qp_id < num_qps; ++qp_id)
  {
    rdma.stop_recv(qp_id);
  }


  int sum = 0;
  for (size_t idx = 0; idx < num_qps; ++idx) {
    sum = sum + results[idx];
  }
  std::cout << sum << std::endl;


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