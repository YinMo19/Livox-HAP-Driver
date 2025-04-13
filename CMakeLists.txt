cmake_minimum_required(VERSION 3.5)
project(livox_hap_driver)

# 启用 Rust 支持（需安装 FindRust.cmake 或手动配置）
find_package(Rust REQUIRED)

# 定义 Rust 构建目标
add_custom_target(
  your_rust_target ALL
  COMMAND cargo build --release
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Building Rust code"
)

# 确保编译命令包含 Rust 路径（可选）
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/target/release)
