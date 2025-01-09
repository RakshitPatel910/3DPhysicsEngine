cmake_minimum_required(VERSION 3.10)

project(3DPhysicsEngine)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)

# Include directories
include_directories(
    src/Include
    C:/msys64/mingw64/include/SDL2    # SDL2 include directory
    C:/msys64/mingw64/include/GL      # OpenGL include directory
    C:/msys64/mingw64/include/GLFW    # GLFW include directory
    C:/msys64/mingw64/include/eigen3    # Eigen include directory
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
file(GLOB_RECURSE SRC_FILES "D:/Projects/3DPhysicsEngine/src/core/*.h" "D:/Projects/3DPhysicsEngine/src/core/*.cpp")

# Main source file
set(MAIN_SRC "D:/Projects/3DPhysicsEngine/src/core/main.cpp")

# Add executable
add_executable(3DPhysicsEngine ${SRC_FILES} ${MAIN_SRC})

# Link libraries
target_link_libraries(3DPhysicsEngine
    mingw32
    SDL2main
    SDL2
    OpenGL::GL   # OpenGL library
    OpenGL32
    freeglut
    glew32
    # eigen3
)

# Additional linking if necessary (Uncomment if GLFW is manually set)
# target_link_libraries(3DPhysicsEngine glfw3)
