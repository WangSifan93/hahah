#!/bin/bash

# 限定目录结构为 apps/e2e_pnc/pnc_interface

# 增量编译功能的构建函数
build() {
  local DMY_PLATFORM=$1
  local DCMAKE_EXPORT_COMPILE_COMMANDS=$2
  local DCMAKE_TOOLCHAIN_FILE=$3
  local DCMAKE_INSTALL_PREFIX=$4
  local IS_DEBUG=$5

  # 根据 is_debug 设置 CMAKE_BUILD_TYPE
  if [ $IS_DEBUG = "ON" ]; then
      BUILD_TYPE="Debug"
  else
      BUILD_TYPE="Release"
  fi

  # 创建构建目录并进入
  mkdir -p build
  cd build

  if [[ -f CMakeCache.txt && $(grep -c "CMakeCache.txt" CMakeCache.txt) -eq 0 ]]; then
      echo "Previous compilation failed, clearing cache and reconfiguring..."
      rm -f CMakeCache.txt
  fi

  # 检查是否需要清除缓存
  if [[ -f CMakeCache.txt ]]; then
      # 获取当前 CMakeCache.txt 中的 CMAKE_BUILD_TYPE
      CURRENT_BUILD_TYPE=$(grep -oP '(?<=^CMAKE_BUILD_TYPE:STRING=).*' CMakeCache.txt)
      # 如果当前构建类型与目标类型不一致，则清除缓存
      if [[ "$CURRENT_BUILD_TYPE" != "$BUILD_TYPE" ]]; then
          echo "Build type change, current: $CURRENT_BUILD_TYPE, next: $BUILD_TYPE"
          echo "CMakeCache.txt found, but the build type is different. Clearing cache..."
          rm -f CMakeCache.txt
      fi
  else
      echo "No CMakeCache.txt found. Proceeding with configuration..."
  fi

  # 如果 CMake 配置文件（CMakeCache.txt）已经存在，则跳过配置过程，直接编译
  if [[ -f CMakeCache.txt ]]; then
    echo "CMakeCache.txt found. Skipping configuration step."
  else
    # 设置 CMake 构建命令
    local CMAKE_CMD="cmake .."

    # 根据 DMY_PLATFORM 设置平台相关选项
    if [[ -n "$DMY_PLATFORM" ]]; then
      CMAKE_CMD="$CMAKE_CMD -DMY_PLATFORM=$DMY_PLATFORM"
    fi

    if [[ "$BUILD_TYPE" == "Debug" ]]; then
      CMAKE_CMD="$CMAKE_CMD -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DCMAKE_CXX_FLAGS_DEBUG=\"-g\""
    else
      CMAKE_CMD="$CMAKE_CMD -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DCMAKE_CXX_FLAGS_RELEASE=\"-O3 -g0\""
    fi

    # 如果指定了编译命令导出
    if [[ "$DCMAKE_EXPORT_COMPILE_COMMANDS" == "1" ]]; then
      CMAKE_CMD="$CMAKE_CMD -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
    fi

    # 如果指定了 toolchain 文件
    if [[ -n "$DCMAKE_TOOLCHAIN_FILE" ]]; then
      CMAKE_CMD="$CMAKE_CMD -DCMAKE_TOOLCHAIN_FILE=$DCMAKE_TOOLCHAIN_FILE"
    fi

    # 设置安装路径
    if [[ -n "$DCMAKE_INSTALL_PREFIX" ]]; then
      CMAKE_CMD="$CMAKE_CMD -DCMAKE_INSTALL_PREFIX=$DCMAKE_INSTALL_PREFIX"
    fi

    # 执行 CMake 配置
    echo "Running CMake: $CMAKE_CMD"
    eval $CMAKE_CMD
  fi

  # 执行增量编译
  echo "Running incremental build...($(($(nproc) - 3)))"
  make -j$(($(nproc) - 3))

  # 如果需要，执行安装
  if [[ -n "$DCMAKE_INSTALL_PREFIX" ]]; then
    echo "Installing the project..."
    make install
  fi
}

main() {
    # 获取当前脚本所在目录的绝对路径
    script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

    # 获取根目录 zark2.0 或 zark2
    apps_dir="$(dirname "$(dirname "$script_dir")")"
    grandparent_dir="$(dirname "$apps_dir")"
    cd $grandparent_dir/apps/e2e_pnc
    # 获取该目录名
    grandparent_name="$(basename "$grandparent_dir")"

    # 检查是否为 zark2.0 或 zark2（不区分大小写）
    if [[ "${grandparent_name,,}" != "zark2.0" && "${grandparent_name,,}" != "zark2" ]]; then
        echo "❌ 错误：项目必须位于名为 'zark2.0' 或 'zark2' 的目录下 (当前为 '$grandparent_name')"
        exit 1
    fi

    # 检查是否为 clean 参数
    if [[ "$1" == "clean" ]]; then
        echo "🧹 清理编译结果..."
        sudo rm -rf "$grandparent_dir/apps/e2e_pnc/build"
        echo "✅ 清理完成"
        exit 0
    fi

    # 设置默认参数
    PLATFORM="x86"
    IS_DEBUG=OFF
    SKIP_FORMAT=OFF

    if [[ "$#" -ge 1 ]]; then
        PLATFORM="$1"
    fi

    if [[ "$#" -ge 2 ]]; then
        if [[ "$2" == "debug" ]]; then
            IS_DEBUG=ON
        elif [[ "$2" == "--nf" ]]; then
            SKIP_FORMAT=ON
        else
            echo "❌ 无效的参数: $2"
            echo "✔️  可用参数: [x86|arm] [debug] [clean] [--nf]"
            exit 1
        fi
    fi

    if [[ "$#" -ge 3 && "$3" == "--nf" ]]; then
        SKIP_FORMAT=ON
    fi

    case "$PLATFORM" in
        x86|arm)
            ;;
        *)
            echo "❌ 无效的参数: $PLATFORM"
            echo "✔️  可用参数: [x86|arm] [debug] [clean] [--nf]"
            exit 1
            ;;
    esac

    # 设置其它参数
    base_path="$grandparent_dir"
    echo "✅ 注册 base_path：$base_path"
    if [[ "$PLATFORM" == "arm" ]]; then
        DCMAKE_TOOLCHAIN_FILE="${base_path}/script/aarch64.cmake"
    else
        DCMAKE_TOOLCHAIN_FILE="${base_path}/script/x86.cmake"
    fi
    DCMAKE_EXPORT_COMPILE_COMMANDS=1
    DCMAKE_INSTALL_PREFIX="${base_path}/install/${PLATFORM}"

    echo "🔧 构建平台：$PLATFORM"
    echo "🔧 Debug模式:$IS_DEBUG"
    echo "🔧 安装路径:$DCMAKE_INSTALL_PREFIX"

    # 检查是否跳过代码格式化
    if [[ "$SKIP_FORMAT" == "OFF" ]]; then
        # 检查系统中是否存在clang-format
        if ! command -v clang-format &> /dev/null; then
            echo "❌ 错误：未找到 clang-format，请安装后重试,安装命令： sudo apt install clang-format"
        else
            # 调用 format.py 格式化代码
            echo "🔧 调用 format.py 格式化代码..."
            /usr/bin/python3 "$grandparent_dir/apps/e2e_pnc/pnc_interface/scripts/format.py"
        fi
    else
        echo "🔧 跳过代码格式化"
    fi

    git config --global --add safe.directory $grandparent_dir/apps/planning/pnc_interface

    cp $grandparent_dir/apps/e2e_pnc/pnc_interface/CMakeLists_e2epnc.txt $grandparent_dir/apps/e2e_pnc/CMakeLists.txt
    build $PLATFORM $DCMAKE_EXPORT_COMPILE_COMMANDS $DCMAKE_TOOLCHAIN_FILE $DCMAKE_INSTALL_PREFIX $IS_DEBUG
    sudo chmod +x $grandparent_dir/install/x86/bin/tros/zoslaunch

    # 显示编译信息
    echo "✅ 编译完成"
    echo "🔧 编译平台：$PLATFORM"
    echo "🔧 编译模式：$(if [[ "$IS_DEBUG" == "ON" ]]; then echo "Debug"; else echo "Release"; fi)"
}

main "$@"

