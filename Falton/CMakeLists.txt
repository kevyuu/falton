file (GLOB_RECURSE SRC src/*.cpp)

file (GLOB_RECURSE SOURCE_FILES src/*.cpp include/*.h)

add_library(falton ${SOURCE_FILES})

add_subdirectory(test)