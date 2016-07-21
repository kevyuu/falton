cmake_minimum_required(VERSION 2.6)
set(CMAKE_LEGACY_CYGWIN_WIN32 0)

project(Falton)

enable_testing()

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR
    "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set(warnings "-Wall -Wextra -Wno-missing-braces -Wno-reorder -Wno-c++11-narrowing -g -O3")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    set(warnings "/W4 /WX /Ehsc")
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
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -stdlib=libc++")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c++11 -stdlib=libc++")
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake_modules")

find_package(OpenGL REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(GLEW REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/Falton/include)
add_subdirectory(Falton)

link_libraries(${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} falton)
include_directories(${GLFW_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS})

include_directories(/usr/local/Cellar/allegro/5.0.11/include)
link_directories(/usr/local/Cellar/allegro/5.0.11/lib)

add_subdirectory(src)

set(CONFIGURED_ONCE TRUE CACHE INTERNAL
        "A flag showing that CMake has configured at least once.")