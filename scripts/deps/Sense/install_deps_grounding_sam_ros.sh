#!/bin/bash

# Function to check command success
check_success() {
    if [ $? -ne 0 ]; then
        echo "Error occurred in the previous command. Exiting."
        exit 1
    fi
}

# Clone grounding_sam_ros
mkdir -p src/perception/
cd src/perception/

if [ -d "grounding_sam_ros" ]; then
    echo "grounding_sam_ros directory already exists. Skipping clone."
else
    echo "Cloning grounding_sam_ros..."
    git clone https://github.com/yutian929/YanBot-Sense_grounding_sam_ros.git grounding_sam_ros
    check_success
fi
# Download the checkpoints
cd grounding_sam_ros/weights/
bash download_ckpts.sh
check_success
cd ../
# Prepare conda environment
# 检查 Conda 是否安装
if ! command -v conda &> /dev/null; then
    echo "Error: Conda is not installed. Please install Conda first."
    exit 1
fi

# 检查环境是否存在
ENV_NAME="gsam"
if conda env list | grep -q -E "\b${ENV_NAME}\b"; then
    echo "Conda environment '$ENV_NAME' already exists."
else
    echo "Creating conda environment from gsam.yaml..."
    conda env create -f gsam.yaml
    check_success
fi

# 激活 Conda 环境（需要先初始化 shell）
# eval "$(conda shell.bash hook)"
# conda init
conda activate $ENV_NAME
check_success

# Install GroundingDINO
echo "Installing GroundingDINO..."
if [ -d "GroundingDINO" ]; then
    echo "GroundingDINO directory already exists. Skipping clone."
else
    echo "Cloning GroundingDINO..."
    git clone https://github.com/IDEA-Research/GroundingDINO.git GroundingDINO
    check_success
fi
cd GroundingDINO/
pip install -e .
cd ..
# rm -rf GroundingDINO/  # 安装完成后删除源码
check_success

# Install SAM
echo "Installing SAM..."
python -m pip install git+https://github.com/facebookresearch/segment-anything.git
check_success


cd ../
cd ../../
