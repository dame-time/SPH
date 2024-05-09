cmake_minimum_required(VERSION 3.26)
project(SPH)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenGL REQUIRED)
find_package(glfw3 3.3 REQUIRED)

include_directories(
        ${OPENGL_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/Includes/glad/include
        ${CMAKE_SOURCE_DIR}/Renderer
        ${CMAKE_SOURCE_DIR}/Physics
        ${CMAKE_SOURCE_DIR}/Shader
        ${CMAKE_SOURCE_DIR}/Math
        ${CMAKE_SOURCE_DIR}/Math/Vector
        ${CMAKE_SOURCE_DIR}/Math/Matrix
        ${CMAKE_SOURCE_DIR}/Math/Rotation
        ${CMAKE_SOURCE_DIR}/Math/Versor
        ${CMAKE_SOURCE_DIR}/Math/Point
)

file(GLOB_RECURSE SOURCES "*.cpp" "*.hpp")

set(SEEN_FILES "")
set(DUPLICATE_FILES "")

foreach(source ${SOURCES})
    if(source IN_LIST SEEN_FILES)
        list(APPEND DUPLICATE_FILES ${source})
    else()
        list(APPEND SEEN_FILES ${source})
    endif()
endforeach()

if(DUPLICATE_FILES)
    message(FATAL_ERROR "Duplicate files found:")
    foreach(duplicate ${DUPLICATE_FILES})
        message(FATAL_ERROR "${duplicate}")
    endforeach()
endif()

foreach(source ${SOURCES})
    if(NOT (${source} MATCHES "/Math/" OR ${source} MATCHES "/Shader/" OR ${source} MATCHES "/Renderer/"
            OR ${source} MATCHES "/Physics/"))
        list(REMOVE_ITEM SOURCES ${source})
    endif()
endforeach()

foreach(source ${SOURCES})
    message(STATUS "Source file: ${source}")
endforeach()

add_executable(SPH ${SOURCES} ${CMAKE_SOURCE_DIR}/Includes/glad/src/glad.c main.cpp
        Physics/SpatialGrid.hpp)
target_compile_options(${PROJECT_NAME} PUBLIC -g -O3 -march=native -flto -funroll-loops -finline-functions -fno-omit-frame-pointer -fno-strict-aliasing -fno-rtti -fno-stack-protector)
#target_compile_options(${PROJECT_NAME} PUBLIC -g)

target_link_libraries(SPH ${OPENGL_LIBRARIES} glfw)
