file (GLOB_RECURSE SRC src/*.cpp)

file (GLOB_RECURSE SOURCE_FILES src/*.cpp include/*.h)

add_library(falton ${SOURCE_FILES} src/physics/joint/ftDynamoJoint.cpp include/falton/physics/joint/ftDynamoJoint.h)

add_subdirectory(test)