add_executable(serial_rpc_tests
  test_msgpack.cpp
  test_framing.cpp
)
target_link_libraries(serial_rpc_tests PRIVATE serial_rpc doctest::doctest)
target_compile_options(serial_rpc_tests PRIVATE -Wall -Wextra -Wpedantic)

include(${doctest_SOURCE_DIR}/scripts/cmake/doctest.cmake)
doctest_discover_tests(serial_rpc_tests)
