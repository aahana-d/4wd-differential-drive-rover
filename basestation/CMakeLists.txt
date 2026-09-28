cmake_minimum_required(VERSION 3.16)
project(rover_telemetry_logger CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SQLITE3 REQUIRED sqlite3)

add_executable(telemetry_logger
    telemetry_logger.cpp
    ../common/Protocol.cpp
)

target_include_directories(telemetry_logger PRIVATE ../common ${SQLITE3_INCLUDE_DIRS})
target_link_libraries(telemetry_logger PRIVATE ${SQLITE3_LIBRARIES})

# copy schema.sql next to the built binary so `applySchema()`'s relative path ("schema.sql") resolves when running from the build directory
add_custom_command(TARGET telemetry_logger POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_CURRENT_SOURCE_DIR}/schema.sql
        $<TARGET_FILE_DIR:telemetry_logger>/schema.sql
)
