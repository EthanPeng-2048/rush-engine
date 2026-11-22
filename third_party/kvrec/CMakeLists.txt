cmake_minimum_required(VERSION 3.20)
project(kvrec VERSION 0.1.0 LANGUAGES CXX)

# ── 依赖：exSTL（Result / EX_* 宏）──────────────────────────────────────────
# 嵌入主仓时目标已由主仓 add_subdirectory(exstl) 创建；standalone 时从 sibling
# ../exstl 拉入。零嵌套 submodule——只消费目标，不 git pin。
if(NOT TARGET exstl::exstl)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../exstl/CMakeLists.txt)
        add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../exstl ${CMAKE_CURRENT_BINARY_DIR}/exstl)
    else()
        find_package(exstl REQUIRED)
    endif()
endif()

# ── 门面：header-only INTERFACE 库 ─────────────────────────────────────────
add_library(kvrec INTERFACE)
add_library(kvrec::kvrec ALIAS kvrec)

target_include_directories(kvrec INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(kvrec INTERFACE cxx_std_23)
target_link_libraries(kvrec INTERFACE exstl::exstl)

# ── 双模式：standalone 才建测试 ────────────────────────────────────────────
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(KVREC_STANDALONE ON)
else()
    set(KVREC_STANDALONE OFF)
endif()

option(KVREC_ENABLE_TESTS "Build kvrec tests" ${KVREC_STANDALONE})

if(KVREC_ENABLE_TESTS)
    enable_testing()
    add_executable(kvrec_test tests/kvrec_test.cpp)
    target_link_libraries(kvrec_test PRIVATE kvrec::kvrec)
    target_compile_features(kvrec_test PRIVATE cxx_std_23)
    add_test(NAME kvrec_test COMMAND kvrec_test)
endif()

# ── 安装（供 find_package / 打包）──────────────────────────────────────────
include(GNUInstallDirs)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS kvrec EXPORT kvrecTargets)
install(EXPORT kvrecTargets
    NAMESPACE kvrec::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kvrec
)
