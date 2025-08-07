#! /bin/bash
cd /zark/release/e2e_pnc/zark/
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:./lib/
export ZLOG_MODULE_NAME="e2e"
export ZLOG_OUTPUT_DIR="/tmp/zlog/"${ZLOG_MODULE_NAME}/
export ZLOG_OUTPUT_MOD=1
export ZLOG_FILE_MAX_SIZE=200
export ZLOG_TO_STD_ERR=1
export ZLOG_SET_COLOR=1
export ZLOG_MIN_LEVEL="info"
export PLATFORM="x86"

pwd && ls

DEBUG_BUILD=0
for arg in "$@"; do
    case $arg in
        --debug)
        DEBUG_BUILD=1
        shift
        ;;
    esac
done

if [ $DEBUG_BUILD -eq 1 ]; then
    gdb --args  zoslaunch -c apps/e2e_pnc/config/e2e_pnc_component/process_x86.json -w .
else
    zoslaunch -c apps/e2e_pnc/config/e2e_pnc_component/process_x86.json -w . 2>&1 | tee /zark/release/e2e_pnc/e2e_noa.log
fi
