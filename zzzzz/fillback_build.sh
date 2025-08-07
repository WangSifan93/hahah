#!/bin/bash -e
cd /zark

bazel build --config=O2 //apps/planning:planning_zark_tarball

folder_path="/zark/release"  
sudo rm -rf $folder_path;

if [ ! -d "$folder_path" ]; then  
    mkdir -p "$folder_path"  
    echo "complete "$folder_path""  
fi  

cp bazel-bin/apps/planning/planning_zark_tarball.tar.gz "$folder_path"

cd "$folder_path"
tar -xzvf planning_zark_tarball.tar.gz  

cd "$folder_path"/zark
cp apps/planning/config/planning_component/communication_x86.json apps/planning/config/planning_component/communication.json  

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:./lib

zoslaunch -c apps/planning/config/planning_component/process.json -w .  
