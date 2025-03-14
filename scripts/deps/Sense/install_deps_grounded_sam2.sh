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

# Clone grounded_sam2 (公共步骤)
mkdir -p src/Cerebellum/
cd src/Cerebellum/

if [ -d "grounded_sam2" ]; then
    echo "grounded_sam2 directory already exists. Skipping clone."
else
    echo "Cloning grounded_sam2..."
    git clone https://github.com/yutian929/YanBot-Grounded_SAM_2.git grounded_sam2
    check_success
fi

# 安装方式选择
echo "请选择安装方式："
echo "1) Docker安装（推荐）"
echo "2) 本地安装"
read -p "请输入数字选择 (1/2): " install_choice

case $install_choice in
    1)
        echo "正在准备Docker安装..."
        cd grounded_sam2/

        # 检查Docker镜像是否存在
        if docker image inspect grounded_sam2:1.0 &> /dev/null; then
            echo "Docker镜像 grounded_sam2:1.0 已存在，跳过构建。"
        else
            echo "开始构建Docker镜像..."
            make build-image
            check_success
        fi

        mkdir -p /tmp/file_pipe/
        sudo chmod 777 /tmp/file_pipe/
        sudo chmod 777 /tmp/file_pipe/*
        check_success
        sudo echo "read" > /tmp/file_pipe/image.flag
        check_success

        cd ../../../
        ;;
    2)
        echo "正在准备本地安装..."
        cd grounded_sam2/
        
        pip3 install torch==2.4.1 torchvision==0.19.1 torchaudio==2.4.1 --index-url https://download.pytorch.org/whl/cu121  # 2.4.1+cu121
        check_success
        sudo apt install nvidia-cuda-toolkit
        check_success

        # 检查 CUDA 环境变量
        if [ -z "$CUDA_HOME" ] || [ -z "$LD_LIBRARY_PATH" ]; then
            echo "错误：CUDA_HOME 或 LD_LIBRARY_PATH 未设置！"
            echo "请执行以下命令设置 CUDA 路径（示例）："
            echo "export CUDA_HOME=/usr/local/cuda-12.1"
            echo "export LD_LIBRARY_PATH=\$CUDA_HOME/lib64:\$LD_LIBRARY_PATH"
            echo "然后重新运行脚本。"
            exit 1
        fi

        echo "CUDA_HOME: $CUDA_HOME"
        echo "LD_LIBRARY_PATH: $LD_LIBRARY_PATH"
        
        pip install --no-build-isolation -e grounding_dino
        pip show groundingdino
        check_success

        pip install --no-build-isolation -e .
        # pip install --no-build-isolation -e ".[notebooks]"  # 适配 Jupyter
        pip show SAM-2
        check_success

        cd grounding_dino/
        pip install -r requirements.txt --verbose
        check_success
        cd ../

        cd ../../../
        ;;
    *)
        echo "无效的选择，请重新运行脚本并输入1或2。"
        exit 1
        ;;
esac

# 公共后续步骤


# 后续指引
echo -e "\n后续步骤："
if [ "$install_choice" == "1" ]; then
    echo "Docker安装指引："
    echo "1. 进入目录: cd src/perception/grounded_sam2/"
    echo "2. 运行容器: make run"
    echo "3. 在容器内执行: cd /home/appuser/Grounded-SAM-2"
    echo "4. 运行依赖安装: bash scripts/install_deps_xxx.sh"
    echo "详细说明请参考: src/perception/grounded_sam2/README.md"
else
    echo "本地安装指引："
    echo "1. 进入目录: cd src/perception/grounded_sam2/"
    echo "2. 根据项目文档执行本地安装步骤"
fi
