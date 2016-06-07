file (GLOB_RECURSE SRC src/*.cpp)

file (GLOB_RECURSE SOURCE_FILES src/*.cpp include/*.h)

add_library(falton ${SOURCE_FILES} src/container/ftRHHashTable.cpp include/falton/container/ftRHHashTable.h include/falton/physics/joint/ftJoint.h src/physics/joint/ftDistanceJoint.cpp include/falton/physics/joint/ftDistanceJoint.h)

add_subdirectory(test)