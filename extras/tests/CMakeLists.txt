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

# serial_rpc.hpp (host-side RPC client, docs/PLAN.md component 5) + its
# LoopbackPort loopback tests, built twice for the same reason as above: the
# LoopbackPort in test_rpc.cpp wraps a real serial_rpc::Server<MockStream>,
# so building against both device-side storage backends exercises the whole
# host<->device round trip on each.
add_executable(serial_rpc_tests_host_fixed test_rpc.cpp)
target_link_libraries(serial_rpc_tests_host_fixed PRIVATE serial_rpc doctest::doctest)
target_compile_options(serial_rpc_tests_host_fixed PRIVATE -Wall -Wextra -Wpedantic)
target_compile_definitions(serial_rpc_tests_host_fixed PRIVATE SERIAL_RPC_USE_STL=0)
doctest_discover_tests(serial_rpc_tests_host_fixed TEST_PREFIX "host-fixed/")

add_executable(serial_rpc_tests_host_stl test_rpc.cpp)
target_link_libraries(serial_rpc_tests_host_stl PRIVATE serial_rpc doctest::doctest)
target_compile_options(serial_rpc_tests_host_stl PRIVATE -Wall -Wextra -Wpedantic)
target_compile_definitions(serial_rpc_tests_host_stl PRIVATE SERIAL_RPC_USE_STL=1)
doctest_discover_tests(serial_rpc_tests_host_stl TEST_PREFIX "host-stl/")

# -----------------------------------------------------------------------------
# rpc_repl_tests: unit tests for extras/host/tools/rpc_repl/json_value.hpp
# (docs/PLAN.md component 7) -- the Value<->JSON conversion and the REPL's
# command-line tokenizer/parse_args(). Links nlohmann_json only, not replxx
# or fmt/cxxopts: json_value.hpp (the header under test) doesn't need them,
# and per docs/PLAN.md "the terminal parts are tested manually".
# -----------------------------------------------------------------------------
add_executable(rpc_repl_tests test_repl_parse.cpp)
target_include_directories(rpc_repl_tests PRIVATE ${CMAKE_SOURCE_DIR}/extras/host/tools/rpc_repl)
target_link_libraries(rpc_repl_tests PRIVATE serial_rpc doctest::doctest nlohmann_json::nlohmann_json)
target_compile_options(rpc_repl_tests PRIVATE -Wall -Wextra -Wpedantic)
doctest_discover_tests(rpc_repl_tests)
