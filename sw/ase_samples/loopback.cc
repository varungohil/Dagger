

#include <assert.h>
#include <future>
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
#include <csignal>
#include <unordered_set>
//#include "CLI11.hpp"
#include "config.h"
#include "defs.h"
#include "rpc_call.h"
// #include "rpc_client.h"
// // #include "rpc_client_pool.h"
// // #include "rpc_types.h"
#include "utils.h"
#include "rdma_qp.h"

static constexpr size_t kClientQPs = 1;
static constexpr size_t kServerQPs = 1;
static constexpr size_t kRpcDelay =
    500000;  // Adjust this value based on your simulation speed.
static constexpr size_t kClientPostSleepTime = 10;
static constexpr char* kClientIP = "192.168.0.1";
static constexpr char* kServerIP = "192.168.0.2";
static constexpr uint16_t kPort = 3136;

static int run_server(std::promise<bool>& init_pr, std::future<bool>& cmpl_ft);
static int run_client();

int main() {
#ifdef NIC_PHY_NETWORK
  std::cout << "The ASE samle can only be run in the loopback mode"
            << std::endl;
  return 1;
#endif

  std::promise<bool> init_pr;
  std::future<bool> init_ft = init_pr.get_future();

  std::promise<bool> cmpl_pr;
  std::future<bool> cmpl_ft = cmpl_pr.get_future();

  // Start server.
  std::thread server_thread = std::thread(&run_server, std::ref(init_pr), std::ref(cmpl_ft));

  // Wait until server is set-up.
  init_ft.wait();

  // Start client.
  std::thread client_thread = std::thread(&run_client);

  // Wait untill client thread is terminated.
  client_thread.join();
  cmpl_pr.set_value(true);

  // Wait until server thread is terminated.
  server_thread.join();
  return 0;
}

static constexpr size_t kServerNicAddress = 0x20000;

static int run_server(std::promise<bool>& init_pr, std::future<bool>& cmpl_ft) {
    int max_qp_pool_size = 10;
    dagger::RDMA rdma(kServerNicAddress, kServerQPs, max_qp_pool_size);
    

    int res = rdma.init_nic(-1, true);
    if (res != 0) return res;

    res = rdma.start_nic();
    if (res != 0) return res;

    uint16_t p_key = 0; 
    uint32_t q_key = 0;
    std::vector<int> results;
    results.resize(kServerQPs);
    for(int i = 0; i < kServerQPs; i++){
        results[i] = 0;
    }

    for (size_t i = 0; i < kServerQPs; ++i) {
        dagger::IPv4 client_addr(kClientIP, kPort);
        int qp_num = rdma.make_qp();

        if ( qp_num == -1) {
        std::cout << "Failed to make queue pair on thread " << i << std::endl;
        exit(1);
        } else {
        std::cout << "Queue  Pair created on thread " << i << std::endl;
        }

        if (rdma.connect_qp(qp_num, i, client_addr, qp_num, p_key, q_key) != 0) {
        std::cout << "Failed to open connection on thread "<< i << std::endl;
        exit(1);
        } else {
        std::cout << "Connection is open on thread " << i << std::endl;
        }

        rdma.add_recv_queue_entry(qp_num, &results[i], sizeof(int));
        rdma.recv(qp_num);
    }

    init_pr.set_value(true);
    cmpl_ft.wait();

    for (size_t qp_id = 0; qp_id < kServerQPs; ++qp_id)
    {
        rdma.stop_recv(qp_id);
    }


    int sum = 0;
    for (size_t idx = 0; idx < kServerQPs; ++idx) {
        sum = sum + results[idx];
    }
    std::cout << "Sum : " << sum << std::endl;

    res = rdma.stop_nic();
    if (res != 0) return res;

    sleep(10);

    return 0;
}

static constexpr size_t kClientNicAddress = 0x00000;

static int run_client() {
    int max_qp_pool_size = 10;
    dagger::RDMA rdma_client(kClientNicAddress, kClientQPs, max_qp_pool_size);

    int res = rdma_client.init_nic(-1, false);
    if (res != 0) return res;

    res = rdma_client.start_nic();
    if (res != 0) return res;

    uint16_t p_key = 0; 
    uint32_t q_key = 0;

    std::vector<std::thread> threads;
    for (size_t qp_id = 0; qp_id < kClientQPs; ++qp_id) {
        std::cout << "Creating thread " << qp_id << std::endl;
        dagger::IPv4 server_addr(kServerIP, kPort);

        int qp_num = rdma_client.make_qp();
        if ( qp_num == -1) {
            std::cout << "Failed to make queue pair on thread " << qp_id << std::endl;
            exit(1);
        } else {
            std::cout << "Queue  Pair created on thread " << qp_id << std::endl;
        }

        if (rdma_client.connect_qp(qp_num, qp_id, server_addr, qp_id, p_key, q_key) != 0) {
            std::cout << "Failed to open connection on thread "<< qp_id << std::endl;
            exit(1);
        } else {
            std::cout << "Connection is open on thread " << qp_id << std::endl;
        }
        int op1 = qp_id;
        int op2 = qp_id + 1;
        int res = op1 + op2;
        rdma_client.add_send_queue_entry(qp_num, &res, sizeof(res));
        rdma_client.send(qp_num);
    }
    res = rdma_client.stop_nic();
    if (res != 0) return res;
    sleep(10);

    return 0;
}
