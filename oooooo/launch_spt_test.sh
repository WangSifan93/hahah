#!/bin/bash

# 设置核心转储
ulimit -c unlimited

# 日志相关配置
export ZLOG_MODULE_NAME="e2e_pnc_test"
export ZLOG_FILE_MAX_SIZE=2
export ZLOG_TO_STD_ERR=1
export ZLOG_SET_COLOR=1
export ZLOG_MIN_LEVEL="info"
export ZLOG_FILE_ROTATE=1
export ZLOG_OUTPUT_DIR="${PWD}/test_logs/"

# 配置测试二进制路径（根据实际位置修改）
TEST_BIN_RELATIVE_PATH="build/pnc_interface/spt_test/run_unit_tests"  # 示例路径，请根据实际路径修改
TEST_BIN="/zark2/apps/e2e_pnc/${TEST_BIN_RELATIVE_PATH}"

# 检查测试二进制是否存在
if [ ! -f "${TEST_BIN}" ]; then
    echo -e "\033[31m错误：未找到测试可执行文件\033[0m"
    echo "预期路径: ${TEST_BIN}"
    echo "请先编译测试程序，或修改脚本中的 TEST_BIN_RELATIVE_PATH"
    exit 1
fi
LIB_BIN_PATH="/zark2/install/x86"

# 设置库路径
LIBRARY_PATHS=(
    "${LIB_BIN_PATH}/lib/myabsl"
    "${LIB_BIN_PATH}/lib/osqp"
    "${LIB_BIN_PATH}/lib/eigen3"
    "${LIB_BIN_PATH}/lib/gtest"
    "${LIB_BIN_PATH}/lib/protobuf"
    "${LIB_BIN_PATH}/lib/glog"
    "${LIB_BIN_PATH}/lib/gflags"
    "${LIB_BIN_PATH}/lib/fmt"
    "${LIB_BIN_PATH}/lib/ceres"
    "${LIB_BIN_PATH}/lib/nlohmann"
    "${LIB_BIN_PATH}/lib/ap_services"
    "${LIB_BIN_PATH}/lib/system_monitor"
    "${LIB_BIN_PATH}/lib/tros"
    "${LIB_BIN_PATH}/lib/messages"
    "${LIB_BIN_PATH}/lib/zlog"
    "${LIB_BIN_PATH}/lib/boost"
    "${LIB_BIN_PATH}/lib/nvinfer"
    "${LIB_BIN_PATH}/lib/map_service"
    "${LIB_BIN_PATH}/lib/e2e_pnc"
)

# 验证并构建LD_LIBRARY_PATH
valid_paths=()
for path in "${LIBRARY_PATHS[@]}"; do
    if [ -d "${path}" ]; then
        valid_paths+=("${path}")
    else
        echo -e "\033[33m警告：库目录不存在 - ${path}\033[0m"
    fi
done

IFS=":" eval 'joined_paths="${valid_paths[*]}"'
export LD_LIBRARY_PATH="${joined_paths}:${LD_LIBRARY_PATH}"
echo -e "\033[34m库搜索路径:\033[0m\n${LD_LIBRARY_PATH//:/\\n}"

# 创建测试输出目录
mkdir -p "${ZLOG_OUTPUT_DIR}"
mkdir -p "${PWD}/test_results/"

# 测试配置
export GTEST_COLOR=1
#export GTEST_OUTPUT="xml:${PWD}/test_results/"
export PLATFORM="x86"

echo -e "\n\033[1;36m=== 开始执行测试 ===\033[0m"
echo -e "测试程序: \033[35m${TEST_BIN}\033[0m"
echo -e "工作目录: \033[35m${PWD}\033[0m"
echo -e "日志目录: \033[35m${ZLOG_OUTPUT_DIR}\033[0m\n"

# 执行测试
"${TEST_BIN}" "$@"
TEST_EXIT_CODE=$?

# 显示测试结果
if [ $TEST_EXIT_CODE -eq 0 ]; then
    echo -e "\n\033[1;32m✓ 所有测试通过\033[0m"
else
    echo -e "\n\033[1;31m✗ 测试失败 (退出码: ${TEST_EXIT_CODE})\033[0m"
fi

exit $TEST_EXIT_CODE
