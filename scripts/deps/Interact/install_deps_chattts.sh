#!/bin/bash

# Function to check command success
check_success() {
    if [ $? -ne 0 ]; then
        echo "Error occurred in the previous command. Exiting."
        exit 1
    fi
}

# check whether $YANBOT_WS existed.
check_ws() {
    if [ -z "$YANBOT_WS" ]; then
        echo "YANBOT_WS is not set. Exiting."
        exit 1
    else 
        echo "YANBOT_WS is set to $YANBOT_WS"
        cd $YANBOT_WS
    fi
}

# check and goto YANBOT_WS
check_ws

# 安装音频后端依赖
sudo apt install sox libsox-dev
check_success

# Prepare conda environment
# 检查 Conda 是否安装
if ! command -v conda &> /dev/null; then
    echo "Error: Conda is not installed. Please install Conda first."
    exit 1
fi

# 检查环境是否存在
cd src/Cerebellum/tts_pkg/
ENV_NAME="tts"
if conda env list | grep -q -E "\b${ENV_NAME}\b"; then
    echo "Conda environment '$ENV_NAME' already exists."
else
    echo "Creating conda environment from tts.yaml..."
    conda env create -f tts.yaml
    check_success
fi

# 激活 Conda 环境（需要先初始化 shell）
# eval "$(conda shell.bash hook)"
# conda init
conda activate $ENV_NAME
check_success

check_ws
