cmake_minimum_required(VERSION 3.10)

project(test)

set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置源文件
set(SOURCE_FILES src/ja_motor_control.cpp src/ja_multi_motor_control.cpp test.cpp)
include_directories(${CMAKE_SOURCE_DIR}/include)



# 设置平台相关的配置
if (WIN32) # Windows 配置
    # 包含头文件路径
    include_directories(C:/Users/Administrator/serial/include)
    # 链接库文件路径
    link_directories(C:/Users/Administrator/serial/build/Release)
    # 查找 serial 库
    find_library(SERIAL_LIB serial HINTS C:/Users/Administrator/serial/build/Release)
else() # Linux 配置
    # 包含头文件路径
    include_directories(include /usr/local/include)
    # 链接库文件路径
    link_directories(/usr/local/lib)
    # 查找 serial 库
    find_library(SERIAL_LIB serial HINTS /usr/local/lib)

    # # 查找 serial 库
    # find_library(SERIAL_LIB serial REQUIRED)
endif()

# 如果找不到 serial 库，则提示错误
if(NOT SERIAL_LIB)
    message(FATAL_ERROR "Could not find the serial library!")
endif()

# 生成可执行文件并链接 serial 库
add_executable(test ${SOURCE_FILES})
target_link_libraries(test ${SERIAL_LIB})
