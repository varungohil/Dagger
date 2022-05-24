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
#include <chrono>
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

int kFpgaBus = 0xaf;
static constexpr uint64_t kNicAddress = 0x00000;
/// Networking configuration.
static constexpr char* kServerIP = "10.212.62.191";
static constexpr uint16_t kPort = 12345;


static double rdtsc_in_ns() {
  uint64_t a = dagger::utils::rdtsc();
  sleep(1);
  uint64_t b = dagger::utils::rdtsc();

  return (b - a) / 1000000000.0;
}


int add_num(dagger::RDMA* rdma, dagger::IPv4 server_addr, int remote_qp_num, int p_key, uint32_t q_key, int thread_id, int op1, int op2)
{
  int qp_num = rdma->make_qp();
  if ( qp_num == -1) {
    std::cout << "Failed to make queue pair on thread " << thread_id << std::endl;
    exit(1);
  } else {
    std::cout << "Queue  Pair created on thread " << thread_id << std::endl;
  }
  std::cout << "About to connect_qp" << std::endl;
  if (rdma->connect_qp(qp_num, thread_id, server_addr, remote_qp_num, p_key, q_key) != 0) {
    std::cout << "Failed to open connection on thread "<< thread_id << std::endl;
    exit(1);
  } else {
    std::cout << "Connection is open on thread " << thread_id << std::endl;
  }
  //int send_data;
  //int one = 1;
  //int two = 2;
  //int three = 3;
  //int four = 4;
  //int five = 5;
  //int six = 6;
  //int seven = 7;
  //int eight = 8;
  //int nine = 9;
  //int ten = 10; 
  std::vector<int> send_vec;
  for(int data = op1; data <= op2; data++){
    send_vec.push_back(data);
  } 
  for(int i = 0; i < send_vec.size(); i++){
    rdma->add_send_queue_entry(qp_num, &send_vec[i], sizeof(send_vec[i]));
  }
 //for(int num = op1; num < op2; num++){
  //   send_data = num;
  //   std::cout << "Added sendq entry for " << send_data << std::endl;
  //   rdma->add_send_queue_entry(qp_num, &send_data, sizeof(send_data));
  //}
    
    // rdma->add_send_queue_entry(qp_num, &one, sizeof(send_data));
    // rdma->add_send_queue_entry(qp_num, &two, sizeof(send_data));
    // rdma->add_send_queue_entry(qp_num, &three, sizeof(send_data));
    // rdma->add_send_queue_entry(qp_num, &four, sizeof(send_data));
    // rdma->add_send_queue_entry(qp_num, &five, sizeof(send_data));
 
    // rdma->add_send_queue_entry(qp_num, &six, sizeof(send_data));
    // rdma->add_send_queue_entry(qp_num, &seven, sizeof(send_data));
    // rdma->add_send_queue_entry(qp_num, &eight, sizeof(send_data));
    // rdma->add_send_queue_entry(qp_num, &nine, sizeof(send_data));
    // rdma->add_send_queue_entry(qp_num, &ten, sizeof(send_data));
 
  //for(int num = op1; num < op2; num++){
  //   std::cout << "Sending " << num << std::endl;
  //   rdma->send(qp_num);
  //}
  sleep(30);  
  for(int i = 0; i < send_vec.size(); i++){
   sleep(1);
   auto tp = std::chrono::high_resolution_clock::now();
   auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch()).count();
   std::cout << "Sent at " << nanos << std::endl;
   rdma->send(qp_num);  
  }
  int res = op1 + op2;
  //std::cout << "Adding send queue entry" << std::endl;
  //rdma->add_send_queue_entry(qp_num, &res, sizeof(res));
  //std::cout << "Sending" << std::endl;
  //rdma->send(qp_num);
  //std::cout << "Sent! Now returning" << std::endl;
  return res;
}

int main(int argc, char* argv[]) {
  // Parse input.
  // CLI::App app{"Benchmark Client"};

  size_t num_of_threads = 1;
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
  //res = rdma.run_perf_thread({true, true, true, true, true}, nullptr);
  //if (res != 0) return res;

  uint16_t p_key = 0; 
  uint32_t q_key = 0;
  // Run benchmarking threads.
  //std::vector<std::thread> threads;
  //for (size_t thread_id = 0; thread_id < num_of_threads; ++thread_id) {
    //std::cout << "Creating thread " << thread_id << std::endl;
    // Open connection
  dagger::IPv4 server_addr(kServerIP, kPort);

   // q_key = thread_id;

    // Run the benchmarking thread on the client rpc_client
  int op1 = 0;
  int op2 = 20;
  int remote_qp_num = 0;
  int qp_num = rdma.make_qp();
  if ( qp_num == -1) {
    std::cout << "Failed to make queue pair "  << std::endl;
    exit(1);
  } else {
    std::cout << "Queue  Pair created" << std::endl;
  }
  std::cout << "About to connect_qp" << std::endl;
  if (rdma.connect_qp(qp_num, 0, server_addr, remote_qp_num, p_key, q_key) != 0) {
    std::cout << "Failed to open connection" << std::endl;
    exit(1);
  } else {
    std::cout << "Connection is open" << std::endl;
  }

  std::vector<int> send_vec;
  for(int data = op1; data <= op2; data++){
    send_vec.push_back(data);
  } 
  for(int i = 0; i < send_vec.size(); i++){
    rdma.add_send_queue_entry(qp_num, &send_vec[i], sizeof(send_vec[i]));
  }

  for(int i = 0; i < send_vec.size(); i++){
   sleep(1);
   auto tp = std::chrono::high_resolution_clock::now();
   auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch()).count();
   std::cout << "Sent at " << nanos << std::endl;
   rdma.send(qp_num);  
  }
    //std::thread thr = std::thread(&add_num, &rdma, server_addr, thread_id, p_key, q_key, thread_id, op1, op2);
    //std::cout << "Created thread " << thread_id << std::endl;
    //threads.push_back(std::move(thr));
    //std::cout << "Pushed thread " << thread_id << std::endl;
  // }

  // Wait until all the benchmarking threads are completed.
  //for (auto& thr : threads) {
  //  thr.join();
  //}
  sleep(5);
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
