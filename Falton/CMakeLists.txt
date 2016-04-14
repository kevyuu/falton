file (GLOB_RECURSE SRC src/*.cpp)

file (GLOB_RECURSE SOURCE_FILES src/*.cpp include/*.h)

add_library(falton ${SOURCE_FILES} src/physics/ftMassComputer.cpp src/physics/joint/ftPinJoint.cpp include/falton/physics/joint/ftPinJoint.h)

add_subdirectory(test)