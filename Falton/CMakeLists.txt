file (GLOB_RECURSE SRC src/*.cpp)

file (GLOB_RECURSE SOURCE_FILES src/*.cpp include/*.h)

add_library(falton ${SOURCE_FILES} include/falton/setting/general.h include/falton/setting/dynamicBVH.h src/physics/collision/broadphase/ftDynamicBVH.cpp include/falton/physics/collision/broadphase/ftDynamicBVH.h)

add_subdirectory(test)