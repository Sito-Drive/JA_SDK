cmake_minimum_required(VERSION 3.8)

# 项目名称
project(project)

# 如果支持，请为 MSVC 编译器启用热重载
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

# 设置默认构建类型为 Release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 包含头文件目录
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(C:/Users/Administrator/Desktop/JA_485/C++/Windows/serial/include)

# 根据生成配置自动选择库路径和设置编译选项
set(SERIAL_RELEASE_PATH C:/Users/Administrator/Desktop/JA_485/C++/Windows/serial/build/Release)

# 根据构建类型设置编译选项
if (CMAKE_BUILD_TYPE STREQUAL "Release")
    link_directories(${SERIAL_RELEASE_PATH})
    add_compile_definitions(_ITERATOR_DEBUG_LEVEL=0)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD")
elseif (CMAKE_BUILD_TYPE STREQUAL "Debug")
    link_directories(${SERIAL_DEBUG_PATH})
    add_compile_definitions(_ITERATOR_DEBUG_LEVEL=2)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd")
else()
    message(WARNING "Unsupported build type! Defaulting to Release.")
endif()

# 查找 serial 库
find_library(SERIAL_LIB serial HINTS ${SERIAL_DEBUG_PATH} ${SERIAL_RELEASE_PATH})

# 如果找不到 serial 库，则抛出错误
if (NOT SERIAL_LIB)
    message(FATAL_ERROR "Could not find the serial library!")
endif()

# 添加源文件并生成可执行文件
add_executable(project 
    "test.cpp" 
    "include/ja_motor_control.h" 
    "include/ja_multi_motor_control.h" 
    "src/ja_motor_control.cpp" 
    "src/ja_multi_motor_control.cpp"
)

# 链接 serial 库
target_link_libraries(project ${SERIAL_LIB})

# 输出构建类型信息
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Iterator debug level: ${_ITERATOR_DEBUG_LEVEL}")
message(STATUS "Runtime library settings: Debug(${CMAKE_CXX_FLAGS_DEBUG}), Release(${CMAKE_CXX_FLAGS_RELEASE})")
