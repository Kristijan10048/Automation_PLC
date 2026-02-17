cmake_minimum_required(VERSION 3.16)
project(oip_shell LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Source files
add_executable(oip_shell oip_shell.cpp)

# Include headers (local to this directory)
target_include_directories(oip_shell PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Link against pre-built libraries in lib/
set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)

target_link_libraries(oip_shell PRIVATE
    ${LIB_DIR}/plctag.lib
    ${LIB_DIR}/open62541.lib
    ws2_32      # Windows sockets (required by open62541)
    iphlpapi    # IP helper API (required by open62541)
)

# Copy DLLs to output directory so the executable can find them
add_custom_command(TARGET oip_shell POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${LIB_DIR}/plctag.dll
        $<TARGET_FILE_DIR:oip_shell>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${LIB_DIR}/open62541.dll
        $<TARGET_FILE_DIR:oip_shell>
)
