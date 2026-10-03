cmake_minimum_required(VERSION 3.15)
project(obb_chai3d LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CHAI3D_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../Chai3D-Tower-Defense/externals/chai3d")
set(CHAI3D_LIBRARY "${CHAI3D_ROOT}/build/libchai3d.a")

if(NOT EXISTS "${CHAI3D_LIBRARY}")
    message(FATAL_ERROR
        "Khong tim thay thu vien Chai3D: ${CHAI3D_LIBRARY}\n"
        "Hay build Chai3D truoc khi build project nay."
    )
endif()

find_package(OpenGL REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

file(GLOB OBB_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

add_library(obb STATIC ${OBB_SOURCES})

target_include_directories(obb
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${CHAI3D_ROOT}/src"
        "${CHAI3D_ROOT}/externals/Eigen"
        "${CHAI3D_ROOT}/externals/glew/include"
        "${CHAI3D_ROOT}/externals/glfw/include"
        ${LIBUSB_INCLUDE_DIRS}
)

# Đổi PRIVATE thành PUBLIC để obb_tests cũng nhận được 2 macro này
target_compile_definitions(obb PUBLIC LINUX GLEW_STATIC)

add_executable(obb_tests
    tests/main.cpp
    tests/test_math.cpp
    tests/test_traversal.cpp
    tests/test_bench_mark.cpp
)

target_link_directories(obb_tests PRIVATE
    "${CHAI3D_ROOT}/lib/release/lin-x86_64-cc"
)

target_link_libraries(obb_tests
    PRIVATE
        obb
        "${CHAI3D_LIBRARY}"
        OpenGL::GL
        OpenGL::GLU     # <-- BẮT BUỘC: Thêm GLU để sửa triệt để lỗi linker glu*
        ${LIBUSB_LIBRARIES}
        Threads::Threads
        dl
        rt
        asound
        glfw
)