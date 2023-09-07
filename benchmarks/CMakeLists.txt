find_package(benchmark REQUIRED)

macro(ADD_BENCHMARK name)
  add_executable(${name} ${name}.cpp)

  target_link_libraries(${name} PUBLIC state-observation benchmark::benchmark ${ARGN})

  add_test(
    NAME ${name}
    COMMAND $<TARGET_FILE:${name}>
    WORKING_DIRECTORY $<TARGET_FILE_DIR:${name}>)
  # Adding a project configuration file (for MSVC only)
  generate_msvc_dot_user_file(${name})
endmacro()

add_benchmark(bench-covMultSpeed)
