#! /bin/bash

copy_build_file() {
    local folder=$1
    local flag=$2
    local lib_dir=$3
    cd $folder
    if [ "$flag" -eq 1 ]; then
        sudo cp BUILD_ALL $3/BUILD
    else
        sudo cp BUILD_LIB $3/BUILD
    fi
    sudo chmod 777 $3/BUILD
    cd - > /dev/null
}

check_and_create_dir() {
    local dir=$1
    if [ ! -d "$dir" ]; then
        sudo mkdir -p "$dir"
        sudo chmod 777 "$dir"
    fi
}

check_cpp_cc_files() {
    local folder=$1
    if find "$folder" -type f \( -name "*.cpp" -o -name "*.cc" \) | grep -q .; then
        return 0
    else
        return 1
    fi
}

PLATFORM="x86"
DEBUG_BUILD=0
UNKNOWN_OPTION=0
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --debug)
            DEBUG_BUILD=1
            ;;
        x86|orin|thor)
            PLATFORM=$1
            ;;
        *)
            echo -e "\E[1;31mUnknown option:\E[0m $1"
            UNKNOWN_OPTION=1
            ;;
    esac
    shift
done

if [ $UNKNOWN_OPTION -eq 1 ]; then
    echo -e "\E[1;32mUsage:\E[0m $0 [\E[1;34m--debug\E[0m] [\E[1;34mx86\E[0m|\E[1;34morin\E[0m|\E[1;34mthor\E[0m]"
    exit 1
else
    echo -e "============ \E[1;32mPlatform: $PLATFORM\E[0m ============"
fi

BASE_DIR="/zark/apps/e2e_pnc"
RELEASE_LIB_DIR="/zark/release/e2e_pnc/zark/lib"
PLANNER_DEPS_DIR="/zark/apps/e2e_pnc/pnc_interface/deps"
ZARK_DIR="/zark"

ALG_MODULES=("dispatcher_selector" "interactive_speedplan" "pnp_decision" "trajectory_optimization")

process_compile_files() {
    local folder=$1
    check_and_create_dir $BASE_DIR/$folder
    if check_cpp_cc_files $BASE_DIR/$folder; then
        echo "$folder use src compile"
        copy_build_file "$PLANNER_DEPS_DIR/$folder" 1 $BASE_DIR/$folder
    else
        echo "$folder use lib compile"
        cp -r $PLANNER_DEPS_DIR/$folder/include/ $BASE_DIR/$folder/
        cp -r $PLANNER_DEPS_DIR/$folder/$PLATFORM/*.so $BASE_DIR/$folder/
        sudo chmod 777 -R $BASE_DIR/$folder/*
        copy_build_file "$PLANNER_DEPS_DIR/$folder" 0 $BASE_DIR/$folder
    fi
}

check_and_create_dir() {
    local dir=$1
    if [ ! -d "$dir" ]; then
        sudo mkdir -p "$dir"
        sudo chmod 777 "$dir"
    fi
}

cp $BASE_DIR/pnc_interface/BUILD_E2E_PNC $BASE_DIR/BUILD

# copy algorithm files needed by compile
for module in "${ALG_MODULES[@]}"; do
    process_compile_files $module
done

# clean and build
cd /zark/release/e2e_pnc
#bazel clean
#bazel clean --expunge

BUILD_MODE="Invalid"
if [ $DEBUG_BUILD -eq 1 ]; then
    BUILD_MODE="Debug"
    echo -e "============ \E[1;32mDebug Mode\E[0m ============"
    bazel build //apps/e2e_pnc:zark-asw-e2e_pnc --experimental_ui_max_stdouterr_bytes=52428800 --config=dbg --copt=-g
else
    BUILD_MODE="Release"
    echo -e "============ \E[1;32mRelease Mode\E[0m ============"
    bazel build //apps/e2e_pnc:zark-asw-e2e_pnc --copt=-O3 --experimental_ui_max_stdouterr_bytes=52428800
fi

check_and_create_dir /zark/release/e2e_pnc/
sudo rm -r zark/
sudo rm -r /zark/release/e2e_pnc/zark-asw-e2e_pnc.tar.gz
sudo cp -r /zark/bazel-bin/apps/e2e_pnc/zark-asw-e2e_pnc.tar.gz /zark/release/e2e_pnc/
cd /zark/release/e2e_pnc
tar -zxf zark-asw-e2e_pnc.tar.gz
mkdir -p zark/messages
sudo cp -r /zark/messages/communication /zark/release/e2e_pnc/zark/messages/

# update *.so files and include files
for module in "${ALG_MODULES[@]}"; do
    sudo cp "$RELEASE_LIB_DIR/lib${module}.so" "$PLANNER_DEPS_DIR/$module/$PLATFORM/"
    sudo chmod 777 "$PLANNER_DEPS_DIR/$module/$PLATFORM/lib${module}.so"
    cp -r "$BASE_DIR/$module/include" "$PLANNER_DEPS_DIR/$module/"
    rm -f "$BASE_DIR/$module/BUILD"
done

echo -e "\E[1;33m============ BuildInfo:\E[0m \E[1;32m$PLATFORM $BUILD_MODE\E[0m \E[1;33m============\E[0m"
