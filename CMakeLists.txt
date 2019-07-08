cmake_minimum_required(VERSION 3.14.1)

project(LINE-MOD)

include_directories(include)

file(GLOB SOURCES "src/*.cpp")

#TEMPORARY ADJUSTMENT TODO
list(APPEND CMAKE_PREFIX_PATH "C:/Users/Anton/Documents/libraries")

list(APPEND CMAKE_PREFIX_PATH "C:/Users/Anton/Documents/libraries/glew")

#--- Find OpenGL
find_package(OPENGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})
link_directories(${OPENGL_LIBRARY_DIRS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
else()
	message(STATUS "Found OpenGL")
endif(NOT OPENGL_FOUND)

#--- Find OpenGL
find_package(GLM REQUIRED)
include_directories(${GLM_INCLUDE_DIRS})
link_directories(${GLM_LIBRARY_DIRS})
if(NOT GLM_FOUND)
    message(ERROR " GLM not found!")
else()
	message(STATUS "Found GLM")
endif(NOT GLM_FOUND)


#-- Find ASSIMP
find_package(ASSIMP REQUIRED)
include_directories(${ASSIMP_INCLUDE_DIRS})
link_directories(${ASSIMP_LIBRARY_DIRS})
if(NOT ASSIMP_FOUND)
    message(ERROR " ASSIMP not found!")
else()
	message(STATUS "Found Assim")
endif(NOT ASSIMP_FOUND)

#-- Find SDL2
find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})
link_directories(${SDL2_LIBRARY_DIRS})
if(NOT SDL2_FOUND)
    message(ERROR " SDL2 not found!")
else()
	message(STATUS "Found SDL2")
endif(NOT SDL2_FOUND)

#-- Find GLEW
find_package(GLEW REQUIRED)
include_directories(${GLEW_INCLUDE_DIRS})
link_directories(${GLEW_LIBRARY_DIRS})
if(NOT GLEW_FOUND)
    message(ERROR " GLEW not found!")
endif(NOT GLEW_FOUND)

#-- Find freenect2
find_package(freenect2 REQUIRED)
include_directories(${freenect2_INCLUDE_DIRS})
link_directories(${freenect2_LIBRARY_DIRS})
if(NOT freenect2_FOUND)
    message(ERROR "freenect2 not found!")
endif(NOT freenect2_FOUND)


#-- Find OpenCV
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui imgcodecs rgbd videoio calib3d surface_matching aruco)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
if(NOT OpenCV_FOUND)
    message(ERROR "OpenCV not found!")
endif(NOT OpenCV_FOUND)

add_executable(LINE-MOD ${SOURCES})

set_property(TARGET LINE-MOD PROPERTY CXX_STANDARD 17)
set_property(TARGET LINE-MOD PROPERTY CXX_STANDARD_REQUIRED ON)

target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${GLM_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${ASSIMP_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${SDL2_LIBRARIES})
target_link_libraries(${PROJECT_NAME} GLEW::GLEW)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${freenect2_LIBRARIES})
