include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR}/../../src/)
#link_directories(${CMAKE_SOURCE_DIR}/../
include_directories(${CMAKE_SOURCE_DIR}/src/utils)

# Build benchmark client
set(BENCH_CLIENT_SRC client.cc)
add_executable(dagger_benchmark_client ${BENCH_CLIENT_SRC})
target_compile_definitions(dagger_benchmark_client PRIVATE PROFILE_LATENCY=1)
#target_compile_definitions(dagger_benchmark_client PRIVATE VERBOSE_RPCS=1)
target_link_libraries(dagger_benchmark_client -pthread -ldagger)

# Build benchmark server
set(BENCH_SERVER_SRC server.cc)
add_executable(dagger_benchmark_server ${BENCH_SERVER_SRC})
target_compile_definitions(dagger_benchmark_server PRIVATE PROFILE_LATENCY=1)
#target_compile_definitions(dagger_benchmark_server PRIVATE VERBOSE_RPCS=1)
target_link_libraries(dagger_benchmark_server -pthread -ldagger)

