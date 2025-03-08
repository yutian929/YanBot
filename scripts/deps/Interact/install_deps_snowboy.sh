#!/bin/bash

# Function to check command success
check_success() {
    if [ $? -ne 0 ]; then
        echo "Error occurred in the previous command. Exiting."
        exit 1
    fi
}

mkdir -p src/wakeup/wakeup_pkg/scripts/
cd src/wakeup/wakeup_pkg/scripts/

if [ -d "snowboy" ]; then
    echo "snowboy directory already exists. Skipping clone."
else
    echo "Cloning snowboy..."
    git clone https://github.com/yutian929/YanBot-Interact_snowboy.git snowboy
    check_success
fi

cd snowboy/
bash scripts/install_deps_python3.sh
check_success
cd ..

cd ../../../../
