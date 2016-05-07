file (GLOB_RECURSE SRC src/*.cpp)

file (GLOB_RECURSE SOURCE_FILES src/*.cpp include/*.h)

add_library(falton ${SOURCE_FILES} src/physics/ftMassComputer.cpp src/physics/joint/ftPinJoint.cpp include/falton/physics/joint/ftPinJoint.h src/physics/dynamic/ftPhysicsSystem.cpp include/falton/physics/dynamic/ftPhysicsSystem.h src/physics/collision/ftContact.cpp include/falton/physics/collision/ftContact.h src/physics/dynamic/ftContactConstraint.cpp include/falton/physics/dynamic/ftContactConstraint.h src/physics/dynamic/ftIsland.cpp include/falton/physics/dynamic/ftIsland.h src/physics/dynamic/ftIslandSystem.cpp include/falton/physics/dynamic/ftIslandSystem.h src/physics/collision/broadphase/ftUniformGrids.cpp include/falton/physics/collision/broadphase/ftUniformGrids.h src/physics/collision/broadphase/ftHierarchicalGrid.cpp include/falton/physics/collision/broadphase/ftHierarchicalGrid.h)

add_subdirectory(test)