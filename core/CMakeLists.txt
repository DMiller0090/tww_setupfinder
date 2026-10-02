# setupcore: the window's child process (newline-delimited JSON on stdio), the disc and Dolphin
# readers, and the search.

cmake_minimum_required(VERSION 3.20)
project(setupcore CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Added before this file's add_compile_options so the engine keeps its own warning level.
# The submodule when it is checked out, otherwise a sibling tww_engine checkout.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../external/tww_engine/CMakeLists.txt")
  set(TWW_ENGINE_DEFAULT "${CMAKE_CURRENT_SOURCE_DIR}/../external/tww_engine")
else()
  set(TWW_ENGINE_DEFAULT "${CMAKE_CURRENT_SOURCE_DIR}/../../tww_engine")
endif()
set(TWW_ENGINE_DIR "${TWW_ENGINE_DEFAULT}" CACHE PATH
    "The tww_engine checkout this core builds against")
add_subdirectory(${TWW_ENGINE_DIR} ${CMAKE_CURRENT_BINARY_DIR}/tww_engine)

# Engine headers are decomp C++98 and warn under /W4; include them as SYSTEM from this side only.
# /external:W0 is what makes MSVC honour /external:I.
get_target_property(TWW_ENGINE_INCLUDES tww_engine_port INTERFACE_INCLUDE_DIRECTORIES)
set_target_properties(tww_engine_port PROPERTIES
  INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${TWW_ENGINE_INCLUDES}")
if(MSVC)
  add_compile_options(/external:W0)
endif()

if(WIN32)
  set(DOLPHIN_PLATFORM src/dolphin/dolphin_win.cpp)
elseif(APPLE)
  set(DOLPHIN_PLATFORM src/dolphin/dolphin_mac.cpp)
else()
  set(DOLPHIN_PLATFORM src/dolphin/dolphin_linux.cpp)
endif()

if(WIN32)
  set(MACHINE_PLATFORM src/machine/machine_win.cpp)
elseif(APPLE)
  set(MACHINE_PLATFORM src/machine/machine_mac.cpp)
else()
  set(MACHINE_PLATFORM src/machine/machine_linux.cpp)
endif()

if(MSVC)
  add_compile_options(/W4 /EHsc /permissive-)
else()
  add_compile_options(-Wall -Wextra)
endif()

# Same FP contract as tww_engine: positions computed here are handed to the engine, and a
# contracted multiply-add would change their bits.
if(MSVC)
  add_compile_options(/fp:precise)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-ffp-contract=off)
else()
  message(FATAL_ERROR
    "Unknown compiler '${CMAKE_CXX_COMPILER_ID}'. This binary hands positions to tww_engine, "
    "whose arithmetic depends on the compiler not contracting multiply-adds, and there is no "
    "portable way to ask for that. Teach this file the flag for your compiler.")
endif()

add_library(setupcore_lib STATIC
  src/seam/json.cpp
  src/seam/seam.cpp
  src/settings/settings.cpp
  src/geom/mesh.cpp
  src/disc/yaz0.cpp
  src/disc/rarc.cpp
  src/disc/gcm.cpp
  src/disc/disc.cpp
  src/disc/link.cpp
  src/dolphin/dolphin_common.cpp
  src/dolphin/game.cpp
  src/dolphin/collision.cpp
  src/search/corridor.cpp
  src/search/grid.cpp
  src/search/dzb.cpp
  src/search/target.cpp
  src/search/catalogue.cpp
  src/search/camera_type.cpp
  src/search/cam_clear.cpp
  src/search/cup_tape.cpp
  src/search/cup_exit.cpp
  src/search/l_chain.cpp
  src/search/approx.cpp
  src/search/search.cpp
  src/search/verify.cpp
  src/search/profile.cpp
  src/search/checkpoint.cpp
  ${DOLPHIN_PLATFORM}
  ${MACHINE_PLATFORM}
)
target_include_directories(setupcore_lib PUBLIC src)
target_link_libraries(setupcore_lib PUBLIC tww_engine_port)
if(NOT WIN32 AND NOT APPLE)
  find_package(Threads REQUIRED)
  target_link_libraries(setupcore_lib PUBLIC Threads::Threads)
endif()

add_executable(setupcore src/main.cpp)
target_link_libraries(setupcore PRIVATE setupcore_lib)
# DbgHelp and a release PDB let main.cpp's crash handler print a named backtrace.
if(WIN32)
  target_link_libraries(setupcore PRIVATE dbghelp)
endif()
if(MSVC)
  target_compile_options(setupcore PRIVATE $<$<NOT:$<CONFIG:Debug>>:/Zi>)
  target_link_options(setupcore PRIVATE $<$<NOT:$<CONFIG:Debug>>:/DEBUG> $<$<NOT:$<CONFIG:Debug>>:/OPT:REF> $<$<NOT:$<CONFIG:Debug>>:/OPT:ICF>)
  target_compile_options(setupcore_lib PRIVATE $<$<NOT:$<CONFIG:Debug>>:/Zi>)
endif()

enable_testing()
add_executable(setupcore_tests tests/test_core.cpp)
target_link_libraries(setupcore_tests PRIVATE setupcore_lib)
add_test(NAME setupcore_tests COMMAND setupcore_tests)

add_executable(setupcore_search_tests tests/test_search.cpp)
target_link_libraries(setupcore_search_tests PRIVATE setupcore_lib)
add_test(NAME setupcore_search_tests COMMAND setupcore_search_tests)

# Table generators; none ships with the app.
# Writes src/search/calibration.inc.
add_executable(setupcore_calibrate bench/calibrate.cpp)
target_link_libraries(setupcore_calibrate PRIVATE setupcore_lib)

# Writes src/search/cup_exit_table.inc.
add_executable(setupcore_cup_exit bench/cup_exit_table.cpp)
target_link_libraries(setupcore_cup_exit PRIVATE setupcore_lib)

# Writes src/search/plane_table.inc.
add_executable(setupcore_plane_table bench/plane_table.cpp)
target_link_libraries(setupcore_plane_table PRIVATE setupcore_lib)
