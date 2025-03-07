#!/bin/bash

# Function to check command success
check_success() {
    if [ $? -ne 0 ]; then
        echo "Error occurred in the previous command. Exiting."
        exit 1
    fi
}

# Prompt the user to choose installation method
echo "Select the installation method for PaddleX: DEFAULT=2"
echo "1. Docker Container"
echo "2. Pip Installation"
read -p "Enter your choice (1 or 2): " choice

# Validate input
if [[ "$choice" != "1" && "$choice" != "2" ]]; then
    echo "Invalid choice. Please run the script again and select 1 or 2."
    exit 1
fi

# Clone PaddleX
mkdir -p src/perception/
cd src/perception/

if [ -d "paddlex" ]; then
    echo "paddlex directory already exists. Skipping clone."
else
    echo "Cloning paddlex..."
    git clone https://github.com/yutian929/YanBot-Sense_PaddleX.git paddlex
    check_success
fi

echo "Next, will build docker-paddlex, version 3.0.0b2, date 2025-2-4."
echo "Instructions can be found in src/perception/paddlex/REAMDE.md"

cd ../../

if [ "$choice" == "2" ]; then
    # Method 1: Docker Container
    # Configuration
    IMAGE_REPO="ccr-2vdh3abv-pub.cnc.bj.baidubce.com/paddlepaddle/paddle"
    GPU_IMAGE_TAG="3.0.0b2-gpu-cuda11.8-cudnn8.6-trt8.5"  # CUDA 11.8 image
    CPU_IMAGE_TAG="3.0.0b2"                  # CPU image

    # Function to check image existence
    image_exists() {
        sudo docker images -q "${IMAGE_REPO}:${IMAGE_TAG}" | grep -q .
        return $?
    }

    # Detect GPU
    if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
        echo "[INFO] Detected NVIDIA GPU, using GPU image"
        IMAGE_TAG=$GPU_IMAGE_TAG
        GPU_FLAG="--gpus all"
    else
        echo "[INFO] No GPU detected, using CPU image"
        IMAGE_TAG=$CPU_IMAGE_TAG
        GPU_FLAG=""
    fi

    # Check local image
    echo -e "\n[STEP 1/2] Checking local image..."
    if image_exists; then
        echo "[INFO] Found local image: ${IMAGE_REPO}:${IMAGE_TAG}"
        PULL_NEEDED=false
    else
        echo "[INFO] Local image not found, will pull from registry"
        PULL_NEEDED=true
    fi

    # Pull image if needed
    if $PULL_NEEDED; then
        echo -e "\n[PULL] Pulling Docker image: ${IMAGE_REPO}:${IMAGE_TAG}"
        sudo docker pull ${IMAGE_REPO}:${IMAGE_TAG} || { 
            echo "[ERROR] Pull failed"
            exit 1
        }
    else
        echo -e "\n[SKIP] Using existing local image"
    fi

    # Generate run command (根据官方运行示例调整)
    RUN_CMD="sudo docker run -it --name paddlex_dev ${GPU_FLAG} --shm-size=8G --net=host -v \$(pwd):/paddle -p 7777:7777 -p 7778:7778 ${IMAGE_REPO}:${IMAGE_TAG} /bin/bash"

    # Print instructions
    echo -e "\n[STEP 2/2] Run the container with:"
    echo -e "\n\033[32m${RUN_CMD}\033[0m"
    echo -e "\nAfter running, access the container with:"
    echo "   sudo docker exec -it paddlex_dev /bin/bash"

else
    # Method 2: Pip Installation
    # paddlepaddle
    if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
        echo "[INFO] Detected NVIDIA GPU, using GPU version paddlepaddle."
        # gpu，该命令仅适用于 CUDA 版本为 11.8 的机器环境
        python -m pip install paddlepaddle-gpu==3.0.0b2 -i https://www.paddlepaddle.org.cn/packages/stable/cu118/
        # # gpu，该命令仅适用于 CUDA 版本为 12.3 的机器环境
        # python -m pip install paddlepaddle-gpu==3.0.0b2 -i https://www.paddlepaddle.org.cn/packages/stable/cu123/

    else
        echo "[INFO] No GPU detected, using CPU version paddlepaddle."
        python -m pip install paddlepaddle==3.0.0b2 -i https://www.paddlepaddle.org.cn/packages/stable/cpu/

    fi
    check_success

    python -c "import paddle; print(paddle.__version__)"
    check_success

    # paddlex
    cd src/perception/paddlex
    pip install -e .
    check_success
    cd ../../../
fi
