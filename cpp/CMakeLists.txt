# learn_camera_intrinsics - C++ side
#
#   cd cpp && cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j
#
# Everything lands in cpp/build/bin: two figure renderers, six examples and the
# exercise checker.
cmake_minimum_required(VERSION 3.10)
project(learn_camera_intrinsics CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d imgcodecs)
message(STATUS "OpenCV ${OpenCV_VERSION} from ${OpenCV_DIR}")

# --- the library everything else links against ------------------------------
add_library(camintrinsics
  src/intrinsics.cpp
  src/scene.cpp
  src/renderer.cpp
  src/patterns.cpp
  src/plots.cpp
  src/presets.cpp
  src/util.cpp
)
target_include_directories(camintrinsics PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(camintrinsics PUBLIC ${OpenCV_LIBS})
target_compile_options(camintrinsics PRIVATE -Wall -Wextra)

# --- figure renderers -------------------------------------------------------
# Interactive exploration lives in the web viewer (web/index.html); these
# regenerate the figures used by the README and docs.
foreach(fig render_2d render_3d)
  add_executable(${fig} figures/${fig}.cpp)
  target_link_libraries(${fig} PRIVATE camintrinsics)
endforeach()

# --- examples ---------------------------------------------------------------
file(GLOB EXAMPLE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/examples/*.cpp)
foreach(src ${EXAMPLE_SOURCES})
  get_filename_component(name ${src} NAME_WE)
  add_executable(${name} ${src})
  target_link_libraries(${name} PRIVATE camintrinsics)
endforeach()

# --- parity dumper (scripts/check_parity.sh) --------------------------------
add_executable(parity_dump tools/parity_dump.cpp)
target_link_libraries(parity_dump PRIVATE camintrinsics)

# --- exercises --------------------------------------------------------------
add_executable(check_exercises
  exercises/check_main.cpp
  exercises/checks.cpp
  exercises/exercises.cpp
  exercises/solutions.cpp
)
target_link_libraries(check_exercises PRIVATE camintrinsics)

enable_testing()
add_test(NAME exercise_solutions
         COMMAND check_exercises --solutions)
