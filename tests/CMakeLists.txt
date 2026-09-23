add_executable(serial_rpc_tests
  test_msgpack.cpp
  test_framing.cpp
)
target_link_libraries(serial_rpc_tests PRIVATE serial_rpc doctest::doctest)
target_compile_options(serial_rpc_tests PRIVATE -Wall -Wextra -Wpedantic)

include(${doctest_SOURCE_DIR}/scripts/cmake/doctest.cmake)
doctest_discover_tests(serial_rpc_tests)

# server.h (device dispatcher, docs/PLAN.md components 3 and 3b) is built
# twice against the same test_server.cpp: once forcing the fixed (no-STL,
# InplaceFn/array-table) backend and once forcing the STL
# (std::function/std::vector-table) backend, so both are exercised on the
# host. TEST_PREFIX keeps the two targets' ctest names distinct even though
# they share doctest TEST_CASE names.
add_executable(serial_rpc_tests_fixed test_server.cpp)
target_link_libraries(serial_rpc_tests_fixed PRIVATE serial_rpc doctest::doctest)
target_compile_options(serial_rpc_tests_fixed PRIVATE -Wall -Wextra -Wpedantic)
target_compile_definitions(serial_rpc_tests_fixed PRIVATE SERIAL_RPC_USE_STL=0)
doctest_discover_tests(serial_rpc_tests_fixed TEST_PREFIX "fixed/")

add_executable(serial_rpc_tests_stl test_server.cpp)
target_link_libraries(serial_rpc_tests_stl PRIVATE serial_rpc doctest::doctest)
target_compile_options(serial_rpc_tests_stl PRIVATE -Wall -Wextra -Wpedantic)
target_compile_definitions(serial_rpc_tests_stl PRIVATE SERIAL_RPC_USE_STL=1)
doctest_discover_tests(serial_rpc_tests_stl TEST_PREFIX "stl/")
