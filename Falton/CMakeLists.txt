file(GLOB MATH_SRC src/math/*.cpp)
file(GLOB PHYSICS_SRC src/physics/*.cpp src/physics/shape/*.cpp)

add_library(falton ${MATH_SRC} ${PHYSICS_SRC})

add_subdirectory(test)