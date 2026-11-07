# SIL harness: native build of the servo core + plant sim + sim_runner.
# The core is pure C++ (zero IDF includes) — this build proves it (plan.md
# Phase 5 hard requirement: natively compilable on macOS).
#
#   cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build
#
# SLIDER_SIL_BUGS arms the seeded-bug hooks for the tripwire negative
# controls; production firmware never defines it.

cmake_minimum_required(VERSION 3.20)
project(slider_sil CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CORE ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/components/slider_servo)

add_executable(sim_runner
  sim_runner.cpp
  ident_mode.cpp
  ${CORE}/controller.cpp
  ${CORE}/ident.cpp
)
target_compile_definitions(sim_runner PRIVATE SLIDER_SIL_BUGS)
target_compile_options(sim_runner PRIVATE -Wall -Wextra)

# a bug-free build of the same sources: proves the hooks compile out and
# doubles as the acceptance-run binary
add_executable(sim_runner_prod
  sim_runner.cpp
  ident_mode.cpp
  ${CORE}/controller.cpp
  ${CORE}/ident.cpp
)
target_compile_options(sim_runner_prod PRIVATE -Wall -Wextra)
