/**
 * @file connection_manager.h
 * @brief Implementation of the connection manager driver.
 * @author Nikita Lazarev
 */
#ifndef _CONNECTION_MANAGER_H_
#define _CONNECTION_MANAGER_H_

#include <deque>
#include <map>
#include <utility>

#include "defs.h"

namespace dagger {

//
// Types
//
typedef uint32_t ConnectionId;
typedef uint16_t ConnectionFlowId;

//
class ConnectionManager {
 public:
  ConnectionManager();
  ConnectionManager(size_t max_connections);

  int open_connection(ConnectionId& c_id, const IPv4& dest_addr,
                      ConnectionFlowId flow_id);
  int add_connection(ConnectionId c_id, const IPv4& dest_addr,
                     ConnectionFlowId flow_id, , uint16_t remote_qp_num, uint16_t p_key, uint32_t q_key);
  int close_connection(ConnectionId c_id);

  void dump_open_connections() const;

 private:
  size_t max_connections_;

  // Pool of connection ids
  // - we need it since for now, the hw only supports
  //   a fixed set of connection ids
  std::deque<ConnectionId> c_id_pool_;

  // Currently open connections
  std::map<ConnectionId, std::tuple<IPv4, ConnectionFlowId, uint16_t, uint16_t, uint32_t>> open_connections_;
};

}  // namespace dagger

#endif
