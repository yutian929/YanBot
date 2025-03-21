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

# Clone grounding_sam_ros
mkdir -p src/Cerebellum/
cd src/Cerebellum/

if [ -d "yolo_evsam_ros" ]; then
    echo "yolo_evsam_ros directory already exists. Skipping clone."
else
    echo "Cloning yolo_evsam_ros..."
    git clone https://github.com/zjy109/YanBot-yolo_evsam_ros.git yolo_evsam_ros
    check_success
fi

# 检查 Conda 是否安装
if ! command -v conda &> /dev/null; then
    echo "Error: Conda is not installed. Please install Conda first."
    exit 1
fi

# 检查环境是否存在
ENV_NAME="yoesam"
if conda env list | grep -q -E "\b${ENV_NAME}\b"; then
    echo "Conda environment '$ENV_NAME' already exists."
else
    echo "Creating conda environment from yoesam.yaml..."
    conda env create -f yoesam.yaml
    check_success
fi


# Download the checkpoints
cd yolo_evsam_ros/weights/
bash download_weights.sh
check_success
cd ../

# 激活 Conda 环境（需要先初始化 shell）
eval "$(conda shell.bash hook)"
conda init
conda activate $ENV_NAME
check_success

# Install SAM
echo "Installing SAM..."
python -m pip install git+https://github.com/facebookresearch/segment-anything.git
check_success


cd ../
cd ../../



