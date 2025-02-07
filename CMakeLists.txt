cmake_minimum_required(VERSION 3.10)

project(3DPhysicsEngine)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)

# Include directories
include_directories(
    src/include
    # C:/msys64/mingw64/include/SDL2    # SDL2 include directory
    # C:/msys64/mingw64/include/GL      # OpenGL include directory
    # C:/msys64/mingw64/include/GLFW    # GLFW include directory
    # C:/msys64/mingw64/include/eigen3    # Eigen include directory
    src/include/glad
    src/include/Eigen
)

# Link directories
link_directories(
    C:/msys64/mingw64/lib  # Link directory for libraries
)

# Find OpenGL package
find_package(OpenGL REQUIRED)

# Find GLFW package (manually specify path if necessary)
# Uncomment and modify the line below if GLFW is not found automatically
# set(GLFW_DIR "C:/msys64/mingw64/share/cmake/GLFW")  # Uncomment and adjust if needed
# find_package(GLFW3 REQUIRED)

# Recursively find all .cpp and .h files in src/core
file(GLOB_RECURSE SRC_FILES "D:/Projects/3DPhysicsEngine/src/core/*.h" "D:/Projects/3DPhysicsEngine/src/core/*.cpp" "D:/Projects/3DPhysicsEngine/src/rendering/*.h")
file(GLOB_RECURSE GLAD_SOURCES "src/include/glad/glad.c")

# Main source file
# set(MAIN_SRC "D:/Projects/3DPhysicsEngine/src/core/main.cpp")
set(MAIN_SRC "D:/Projects/3DPhysicsEngine/src/rendering/mainApp.cpp")

# Add executable
add_executable(3DPhysicsEngine ${SRC_FILES} ${MAIN_SRC} ${GLAD_SOURCES})

# Link libraries
target_link_libraries(3DPhysicsEngine
    mingw32
    # SDL2main
    # SDL2
    OpenGL::GL   # OpenGL library
    OpenGL32
    freeglut
    glfw3
    # glew32
    # glad
    # eigen3
)

# Additional linking if necessary (Uncomment if GLFW is manually set)
# target_link_libraries(3DPhysicsEngine glfw3)



# cmake_minimum_required(VERSION 3.10)
# project(3DPhysicsEngine LANGUAGES CXX)

# set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# # Find packages
# # find_package(SDL2 REQUIRED)
# find_package(OpenGL REQUIRED)
# find_package(glfw3 REQUIRED)
# find_package(GLEW REQUIRED)
# find_package(GLUT REQUIRED)  # If actually using GLUT

# # Eigen is header-only; no need to find, just include
# include_directories(${EIGEN3_INCLUDE_DIR})

# # Add executable
# file(GLOB_RECURSE SRC_FILES "src/core/*.cpp" "src/core/*.h")
# add_executable(3DPhysicsEngine ${SRC_FILES})

# # Include directories (modern approach)
# target_include_directories(3DPhysicsEngine PRIVATE
#     src/Include
#     ${SDL2_INCLUDE_DIRS}
#     ${OPENGL_INCLUDE_DIRS}
#     ${GLFW3_INCLUDE_DIRS}
#     ${GLEW_INCLUDE_DIRS}
# )

# # Link libraries
# target_link_libraries(3DPhysicsEngine PRIVATE
#     # SDL2::SDL2
#     OpenGL::GL
#     glfw
#     GLEW::GLEW
#     GLUT::GLUT  # If using GLUT
#     glad  # If using GLAD, ensure it's properly imported
# )