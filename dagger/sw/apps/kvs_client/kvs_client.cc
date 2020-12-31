#include <unistd.h>

#include <cassert>
#include <cinttypes>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <vector>

#include "rpc_call.h"
#include "rpc_client.h"
#include "rpc_client_pool.h"
#include "rpc_types.h"
#include "utils.h"

// HW parameters
#define NIC_ADDR 0x20000

// Timeout
static constexpr size_t t_out = 1000;

// Ctl-C handler
static volatile int keepRunning = 1;

void intHandler(int dummy) {
    keepRunning = 0;
}

static void shell_loop(frpc::RpcClient* rpc_client);

int main(int argc, char* argv[]) {
    constexpr size_t num_of_threads = 1;

    frpc::RpcClientPool<frpc::RpcClient> rpc_client_pool(NIC_ADDR,
                                                         num_of_threads);

    // Init client pool
    int res = rpc_client_pool.init_nic();
    if (res != 0)
        return res;

    // Start NIC with perf enabled
    res = rpc_client_pool.start_nic();
    if (res != 0)
        return res;

    sleep(1);

    frpc::RpcClient* rpc_client = rpc_client_pool.pop();
    assert(rpc_client != nullptr);

    // Open connection
    frpc::IPv4 server_addr("192.168.0.1", 3136);
    if (rpc_client->connect(server_addr, 0) != 0) {
        std::cout << "Failed to open connection on client" << std::endl;
        exit(1);
    } else {
        std::cout << "Connection is open on client" << std::endl;
    }

    // Run interactive shell
    shell_loop(rpc_client);

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

static void shell_loop(frpc::RpcClient* rpc_client) {
    auto cq = rpc_client->get_completion_queue();

    std::cout << "Welcome to Dagger KVS shell" << std::endl;
    std::cout << "Make sure, Dagger is configured with the batch size of 1"
              << std::endl;

    while (keepRunning) {
        char cmd[100];

        std::cout << "> ";
        std::cin.getline(cmd, sizeof(cmd));

        std::istringstream iss(cmd);
        std::vector<std::string> words((std::istream_iterator<std::string>(iss)),
                                         std::istream_iterator<std::string>());

        if (words.size() < 1) {
            std::cout << "wrong comand" << std::endl;
            continue;
        }

        if (words[0] == "set") {
            // do set
            if (words.size() != 3) {
                std::cout << "wrong format of the `set` comand" << std::endl;
                continue;
            }

            std::string key = words[1];
            std::string value = words[2];

            SetRequest set_req;
            set_req.timestamp = static_cast<uint32_t>(frpc::utils::rdtsc());
            sprintf(set_req.key, key.c_str());
            sprintf(set_req.value, value.c_str());
            rpc_client->set(set_req);

            size_t i = 0;
            while(cq->get_number_of_completed_requests() == 0 && i < t_out) {
                ++i;
                usleep(1000);
            }

            if (cq->get_number_of_completed_requests() == 0) {
                std::cout << "> set did not return anything" << std::endl;
            } else {
                std::cout << "> set returned: "
                          << reinterpret_cast<GetResponse*>(cq->pop_response().argv)->value << std::endl;
            }

        } else if (words[0] == "get") {
            // do get
            if (words.size() != 2) {
                std::cout << "wrong format of the `get` comand" << std::endl;
                continue;
            }

            std::string key = words[1];

            GetRequest get_req;
            get_req.timestamp = static_cast<uint32_t>(frpc::utils::rdtsc());
            sprintf(get_req.key, key.c_str());
            rpc_client->get(get_req);

            size_t i = 0;
            while(cq->get_number_of_completed_requests() == 0 && i < t_out) {
                ++i;
                usleep(1000);
            }

            if (cq->get_number_of_completed_requests() == 0) {
                std::cout << "> get did not return anything" << std::endl;
            } else {
                std::cout << "> get returned: "
                          << reinterpret_cast<GetResponse*>(cq->pop_response().argv)->value << std::endl;
            }

        } else {
            std::cout << "unknows command" << std::endl;
            continue;
        }
    }
}
