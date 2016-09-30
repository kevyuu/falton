cmake_minimum_required(VERSION 2.6)
set(CMAKE_LEGACY_CYGWIN_WIN32 0)

project(Falton)

enable_testing()

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR
    "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set(warnings "-Wall -Wextra -Wno-missing-braces -Wno-reorder -g")
elseif(MSVC)
    set(warnings "/W4 /Ehsc")
endif()

if (NOT CONFIGURED_ONCE)
    set(CMAKE_CXX_FLAGS "${warnings}"
            CACHE STRING "Flags used by the compiler during all build types." FORCE)
    set(CMAKE_C_FLAGS   "${warnings}"
            CACHE STRING "Flags used by the compiler during all build types." FORCE)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/lib)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu++11")
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake_modules")

include_directories(${CMAKE_SOURCE_DIR}/Falton/include)
add_subdirectory(Falton)

find_package(OpenGL REQUIRED)

set (GLFW_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/external/GLFW/include)
set (IMGUI_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/external/ImGui/include)
set (FALTON_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/Falton/include)

set (GLFW_LIBRARY_DIR ${CMAKE_SOURCE_DIR}/external/GLFW/lib)

find_library(GLFW_LIBRARY glfw3 HINTS ${GLFW_LIBRARY_DIR})

file (GLOB IMGUI_SOURCES ${CMAKE_SOURCE_DIR}/external/ImGui/src/*.cpp ${CMAKE_SOURCE_DIR}/external/ImGui/src/*.c)

add_subdirectory(src)

set(CONFIGURED_ONCE TRUE CACHE INTERNAL
        "A flag showing that CMake has configured at least once.")