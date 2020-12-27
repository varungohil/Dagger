#include <unistd.h>

#include <algorithm>
#include <cassert>
#include <cinttypes>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>

#include "rpc_call.h"
#include "rpc_client.h"
#include "rpc_client_pool.h"
#include "rpc_types.h"
#include "utils.h"

// HW parameters
#define NIC_ADDR 0x20000

static int run_set_benchmark(frpc::RpcClient* rpc_client,
                             int thread_id,
                             size_t num_iterations,
                             size_t req_delay,
                             double cycles_in_ns,
                             uint64_t starting_key,
                             const char* dst_file_name,
                             size_t set_get_fraction,
                             size_t set_get_req_delay);

static double rdtsc_in_ns() {
    uint64_t a = frpc::utils::rdtsc();
    sleep(1);
    uint64_t b = frpc::utils::rdtsc();

    return (b - a)/1000000000.0;
}

int main(int argc, char* argv[]) {
    double cycles_in_ns = rdtsc_in_ns();
    std::cout << "Cycles in ns: " << cycles_in_ns << std::endl;

    size_t num_of_threads = atoi(argv[1]);
    size_t num_of_requests = atoi(argv[2]);
    size_t req_delay = atoi(argv[3]);
    const char* dst_file_name = argv[4];
    size_t set_get_fraction = atoi(argv[5]);
    size_t set_get_req_delay = atoi(argv[6]);

    frpc::RpcClientPool<frpc::RpcClient> rpc_client_pool(NIC_ADDR,
                                                         num_of_threads);

    // Init client pool
    int res = rpc_client_pool.init_nic();
    if (res != 0)
        return res;

    // Start NIC with perf enabled
    res = rpc_client_pool.start_nic(true);
    if (res != 0)
        return res;

    sleep(1);

    // Run client threads
    std::vector<std::thread> threads;
    for (int thread_id=0; thread_id<num_of_threads; ++thread_id) {
        frpc::RpcClient* rpc_client = rpc_client_pool.pop();
        assert(rpc_client != nullptr);

        // Open connection
        frpc::IPv4 server_addr("192.168.0.1", 3136);
        if (rpc_client->connect(server_addr, thread_id) != 0) {
            std::cout << "Failed to open connection on client" << std::endl;
            exit(1);
        } else {
            std::cout << "Connection is open on client" << std::endl;
        }

        std::thread thr = std::thread(&run_set_benchmark,
                                      rpc_client,
                                      thread_id,
                                      num_of_requests,
                                      req_delay,
                                      cycles_in_ns,
                                      thread_id*num_of_requests,
                                      dst_file_name,
                                      set_get_fraction,
                                      set_get_req_delay);
        threads.push_back(std::move(thr));
    }

    for (auto& thr: threads) {
        thr.join();
    }

    // Check for HW errors
    res = rpc_client_pool.check_hw_errors();
    if (res != 0)
        std::cout << "HW errors found, check error log" << std::endl;
    else
        std::cout << "No HW errors found" << std::endl;

    // Stop NIC
    res = rpc_client_pool.stop_nic();
    if (res != 0)
        return res;

    return 0;
}

static bool sortbysec(const uint64_t &a, const uint64_t &b) {
    return a < b;
}

static void print_latency(std::vector<uint64_t>& latency_records,
                          size_t thread_id,
                          double cycles_in_ns) {
    std::sort(latency_records.begin(), latency_records.end(), sortbysec);

    std::cout << "***** latency results for thread #" << thread_id
              << " *****" << std::endl;
    std::cout << "  total records= " << latency_records.size() << std::endl;
    std::cout << "  median= "
              << latency_records[latency_records.size()*0.5]/cycles_in_ns
              << " ns" << std::endl;
    std::cout << "  90th= "
              << latency_records[latency_records.size()*0.9]/cycles_in_ns
              << " ns" << std::endl;
    std::cout << "  99th= "
              << latency_records[latency_records.size()*0.99]/cycles_in_ns
              << " ns" << std::endl;
}

static int run_set_benchmark(frpc::RpcClient* rpc_client,
                         int thread_id,
                         size_t num_iterations,
                         size_t req_delay,
                         double cycles_in_ns,
                         uint64_t starting_key,
                         const char* dst_file_name,
                         size_t set_get_fraction,
                         size_t set_get_req_delay) {
    // Make an RPC call (SET)
    std::cout << "----------------- doing SET -----------------" << std::endl;
    for(int i=0; i<num_iterations; ++i) {
        // Set <key, value> <i, i+10>
        rpc_client->set(frpc::utils::rdtsc(),
                        starting_key + i,
                        starting_key + i + 10);

       // Blocking delay to control rps rate
       for (int delay=0; delay<req_delay; ++delay) {
           asm("");
       }
    }

    // Wait a bit
    sleep(5);

    // Print Latencies
    auto cq = rpc_client->get_completion_queue();
    auto latency_records = cq->get_latency_records();
    print_latency(latency_records, thread_id, cycles_in_ns);

    cq->clear_latency_records();

    // Make an RPC call (GET)
    std::cout << "----------------- doing SET/GET -----------------" << std::endl;
    // Get distributions from file
    uint64_t* get_dstrs = new uint64_t[num_iterations];
    std::ifstream dst_file;
    dst_file.open(dst_file_name);
    size_t i = 0;
    std::string line;
    if (dst_file.is_open()) {
        while ( getline (dst_file,line) ) {
            get_dstrs[i++] = atoi(line.c_str());
        }
    } else {
        std::cout << "Failed to open distribution file for GET requests" << std::endl;
    }
    assert(i == num_iterations);

    std::cout << "GET distribution > ";
    for (int i=0; i<100; ++i) {
        std::cout << get_dstrs[i] << " ";
    }
    std::cout << "..." << std::endl;

    for(int i=0; i<num_iterations; ++i) {
        if (i%set_get_fraction != 0) {
            rpc_client->get(frpc::utils::rdtsc(), starting_key + get_dstrs[i]);
        } else {
            rpc_client->set(frpc::utils::rdtsc(),
                            num_iterations + starting_key + i,
                            num_iterations + starting_key + i + 10);
        }

        // Blocking delay to control rps rate
        for (int delay=0; delay<set_get_req_delay; ++delay) {
            asm("");
        }
    }

    // Wait a bit
    sleep(5);

    // Get data
    //size_t cq_size = cq->get_number_of_completed_requests();
    //std::cout << "Thread #" << thread_id
    //          << ": CQ size= " << cq_size << std::endl;
    //for (int i=0; i<cq_size; ++i) {
    //    std::cout << cq->pop_response().ret_val << std::endl;
    //}

    // Print Latencies
    latency_records = cq->get_latency_records();
    print_latency(latency_records, thread_id, cycles_in_ns);

    delete[] get_dstrs;
    return 0;
}