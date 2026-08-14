cmake_minimum_required(VERSION 3.15)
project(ecopilot C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# vcpkg toolchain automatically finds SDL2 via find_package
find_package(SDL2 REQUIRED)

add_executable(ecopilot
    src/main.c
    src/vehicle.c
    src/sensor.c
    src/energy.c
)

target_include_directories(ecopilot PRIVATE
    include
    ${SDL2_INCLUDE_DIRS}
)

target_link_libraries(ecopilot PRIVATE
    SDL2::SDL2
    SDL2::SDL2main
)
