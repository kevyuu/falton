file (GLOB_RECURSE SOURCE_FILES src/*.cpp include/*.h)

file (GLOB COLLISION_SOURCES src/collision/*.cpp include/falton/collision/*.h)
file (GLOB BROADPHASE_SOURCES src/collision/broadphase/*.cpp include/falton/collision/broadphase/*.h)
file (GLOB CONTAINER_SOURCES src/container/*.cpp include/falton/container/*.h)
file (GLOB DYNAMIC_SOURCES src/dynamic/*.cpp include/falton/dynamic/*.h)
file (GLOB JOINT_SOURCES src/joint/*.cpp include/falton/joint/*.h)
file (GLOB SHAPE_SOURCES src/shape/*.cpp include/falton/shape/*.h)

add_library(falton ${SOURCE_FILES})

source_group(collision FILES ${COLLISION_SOURCES})
source_group(collision\\broadphase FILES ${BROADPHASE_SOURCES})
source_group(container FILES ${CONTAINER_SOURCES})
source_group(dynamic FILES ${DYNAMIC_SOURCES})
source_group(joint FILES ${JOINT_SOURCES})
source_group(shape FILES ${SHAPE_SOURCES})