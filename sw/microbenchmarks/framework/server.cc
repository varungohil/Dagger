/**
 * @file server.cc
 * @brief Latency/throughput benchmarking server.
 * @author Nikita Lazarev
 */
#include <unistd.h>

#include <csignal>
#include <cstdlib>
#include <iostream>

#include "CLI11.hpp"
#include "config.h"
#include "rpc_call.h"
// #include "rpc_server_callback.h"
// #include "rpc_threaded_server.h"
// #include "rpc_types.h"
#include "rdma_qp.h"

/// HW parameters.
#ifdef PLATFORM_PAC_A10
#  ifdef NIC_PHY_NETWORK
/// Allocate FPGA on bus_2 for the server when running on PAC_A10 with physical
/// networking.
// static constexpr int kFpgaBus = dagger::cfg::platform::pac_a10_fpga_bus_2;
// std::cout << "pac_a10_bus2 - " << kFpgaBus << std::endl;

/// If physical networking, running on different FPGAs, so NIC is placed by
/// 0x20000 for both client and server.
static constexpr uint64_t kNicAddress = 0x20000;

#  else
/// Allocate FPGA on bus_1 for the server when running on PAC_A10 with loopback
/// networking.
// static constexpr int kFpgaBus = dagger::cfg::platform::pac_a10_fpga_bus_1;

/// If loopback, running on the same FPGA, so NIC is placed by 0x00000 for
/// client and 0x20000 for server.
static constexpr uint64_t kNicAddress = 0x20000;

#  endif
#elif PLATFORM_SDP
/// Only loopback is possible here, use skylake_fpga_bus_1 for bus and 0x20000
/// for NIC address.
// static constexpr int kFpgaBus = dagger::cfg::platform::skylake_fpga_bus_1;
// std::cout << "skylake fpga bus 1 - " << kFpgaBus << std::endl;
static constexpr uint64_t kNicAddress = 0x20000;
#else
/// Only loopback is possible here, so -1 for bus and 0x20000 for address.
// static constexpr int kFpgaBus = -1;
static constexpr uint64_t kNicAddress = 0x20000;

#endif

static constexpr int kFpgaBus = 0xaf;
/// Networking configuration.
static constexpr char* kClientIP = "10.212.62.191";
static constexpr uint16_t kPort = 12345;

/// Ctl-C handler.
// static volatile int keepRunning = 1;
// void intHandler(int dummy) { keepRunning = 0; }

// void get_data(dagger::RDMA* rdma, dagger::IPv4 server_addr, int remote_qp_num, int p_key, uint32_t q_key, int qp_id, volatile int* res)
// {
//   int qp_num = rdma->make_qp();
//   if ( qp_num != -1) {
//     std::cout << "Failed to make queue pair on thread " << qp_id << std::endl;
//     exit(1);
//   } else {
//     std::cout << "Queue  Pair created on thread " << qp_id << std::endl;
//   }

//   if (rdma->connect_qp(qp_num, qp_id, server_addr, remote_qp_num, p_key, q_key) != 0) {
//     std::cout << "Failed to open connection on thread "<< qp_id << std::endl;
//     exit(1);
//   } else {
//     std::cout << "Connection is open on thread " << qp_id << std::endl;
//   }

//   rdma->add_recv_queue_entry(qp_num, res, sizeof(res));
//   rdma->recv(qp_num);
  
//   // rdma->stop_recv(qp_num);
// }

static volatile int keep_running = 1;
void initHandler(int dummy) { keep_running = 0;}
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

  size_t num_qps = 1;
  
  //instantiate the RDMA module
  int max_qp_pool_size = num_qps;
  dagger::RDMA rdma(kNicAddress, num_qps, max_qp_pool_size);

  // Initialize the server.
  std::cout << kFpgaBus << std::endl;
  volatile int res = rdma.init_nic(kFpgaBus);
  if (res != 0) return res;

  // Start the server.
  res = rdma.start_nic();
  if (res != 0) return res;

  // Enable perf thread on the nic.
  //res = rdma.run_perf_thread({true, true, true, true, true}, nullptr);
  //if (res != 0) return res;

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

  //int one = 0;
  //int two = 0;
  //int three = 0;
  //int four = 0;
  //int five = 0;
  //int six = 0;
  //int seven = 0;
  //int eight = 0;
  //int nine = 0;
  //int ten = 0;
 
  // Run benchmarking threads.
  uint16_t p_key = 0; 
  uint32_t q_key;
  int op1 = 0;
  int op2 = 20;
  std::vector<int> results;
  results.resize(op2 - op1 + 1);
  for (size_t qp_id = 0; qp_id < num_qps; ++qp_id) {
    // Open connection
    dagger::IPv4 server_addr(kClientIP, kPort + qp_id);
    int qp_num = rdma.make_qp();
    if ( qp_num == -1) {
      std::cout << "Failed to make queue pair on thread " << qp_id << std::endl;
      exit(1);
    } else {
      std::cout << "Queue  Pair created on thread " << qp_id << std::endl;
    }

    q_key = qp_id;
    if (rdma.connect_qp(qp_num, qp_id, server_addr, qp_num, p_key, q_key) != 0) {
      std::cout << "Failed to open connection on thread "<< qp_id << std::endl;
      exit(1);
    } else {
      std::cout << "Connection is open on thread " << qp_id << std::endl;
    }
    //for(int num = 1; num <= 10; num++){
    //   std::cout << &results[num - 1] << std::endl;
    //   rdma.add_recv_queue_entry(qp_num, &results[num-1], sizeof(int)); 
    //}
    //int op1 = 0;
    //int op2 = 10;
    for(int i=0; i < op2 - op1 + 1; i++){
      rdma.add_recv_queue_entry(qp_num, &results[i], sizeof(int) );
    }
    //rdma.add_recv_queue_entry(qp_num, &one, sizeof(int));  
    //rdma.add_recv_queue_entry(qp_num, &two, sizeof(int)); 
    //rdma.add_recv_queue_entry(qp_num, &three, sizeof(int)); 
    //rdma.add_recv_queue_entry(qp_num, &four, sizeof(int)); 
    //rdma.add_recv_queue_entry(qp_num, &five, sizeof(int)); 
    //rdma.add_recv_queue_entry(qp_num, &six, sizeof(int)); 
    //rdma.add_recv_queue_entry(qp_num, &seven, sizeof(int)); 
    //rdma.add_recv_queue_entry(qp_num, &eight, sizeof(int)); 
    //rdma.add_recv_queue_entry(qp_num, &nine, sizeof(int)); 
    //rdma.add_recv_queue_entry(qp_num, &ten, sizeof(int)); 
    
    //std::cout << "one addr = " << &one << std::endl; 
    //std::cout << "two addr = " << &two << std::endl; 
    //std::cout << "three addr = " << &three << std::endl; 
    //std::cout << "four addr = " << &four << std::endl; 
    //std::cout << "five addr = " << &five << std::endl; 
    //std::cout << "six addr = " << &six << std::endl; 
    //std::cout << "seven addr = " << &seven << std::endl; 
    //std::cout << "eight addr = " << &eight << std::endl; 
    //std::cout << "nine addr = " << &nine << std::endl; 
    //std::cout << "ten addr = " << &ten << std::endl; 
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    rdma.recv(qp_num);
    //rdma.add_recv_queue_entry(qp_num, &results[qp_id], sizeof(int));
    //int data_available_count = 0;
    //while(data_available_count <= 10)
    //{
    // if(rdma.is_data_available(qp_num)){
    //       data_available_count++;
    // }
    //}
    //rdma.recv(qp_num);
  }


  //bool all_data_available = 0;
  
  //while(all_data_available == 0){
  //  for (size_t qp_id = 0; qp_id < num_qps; ++qp_id)
  // {
      //std::cout << "Data avaialable = " << rdma.is_data_available(qp_id) << std::endl;
  //    all_data_available = all_data_available || rdma.is_data_available(qp_id);
  //  }
  //}
  signal(SIGINT, initHandler);

  while(keep_running){
    sleep(1);
  } 
  for (size_t qp_id = 0; qp_id < num_qps; ++qp_id)
  {
    rdma.stop_recv(qp_id);
  }

  //std::cout << "One = " << one << std::endl;
  //std::cout << "Two = " << two << std::endl;
  //std::cout << "Three = " << three << std::endl;
  //std::cout << "Four = " << four << std::endl;
  //std::cout << "Five = " << five << std::endl;
  //std::cout << "Six = " << six << std::endl;
  //std::cout << "Seven = " << seven << std::endl;
  //std::cout << "Eight = " << eight << std::endl;
  //std::cout << "Nine = " << nine << std::endl;
  //std::cout << "Ten = " << ten << std::endl;









  int sum = 0;
  for (size_t idx = 0; idx < results.size(); ++idx) {
    std::cout << "results[" << idx << "] = " << results[idx] << std::endl;
    sum = sum + results[idx];
  }
 std::cout << "Sum = " << sum << std::endl;


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
