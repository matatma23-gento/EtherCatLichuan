cmake_minimum_required(VERSION 3.10)
project(EtherCatServoProject)

# Tentukan standar C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Daftarkan folder include utama
include_directories(${CMAKE_SOURCE_DIR}/include)

# (Opsional tapi aman) Tambahkan sub-direktori osal dan oshw agar compiler tidak meleset
include_directories(${CMAKE_SOURCE_DIR}/include/osal)
include_directories(${CMAKE_SOURCE_DIR}/include/oshw)

# Daftarkan folder lib tempat file libsoem.a disimpan
link_directories(${CMAKE_SOURCE_DIR}/lib)

# Tentukan file executable dan file source C++ Anda
add_executable(ethercat_servo src/main.cpp)

# Hubungkan (link) library soem serta library pendukung Linux (pthread, rt)
target_link_libraries(ethercat_servo soem pthread rt)