#!/bin/bash

# Configuration
IMAGE_REPO="paddlecloud/paddlespeech"
GPU_IMAGE_TAG="develop-gpu-cuda11.2-cudnn8-fb4d25"  # CUDA 11.2 image
CPU_IMAGE_TAG="develop-cpu-fb4d25"                  # CPU image

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
RUN_CMD="sudo docker run -it --name paddlespeech_dev ${GPU_FLAG} --net=host -v \$(pwd):/mnt -p 8888:8888 -p 8889:8889 ${IMAGE_REPO}:${IMAGE_TAG} /bin/bash"

# Print instructions
echo -e "\n[STEP 2/2] Run the container with:"
echo -e "\n\033[32m${RUN_CMD}\033[0m"
echo -e "\nAfter running, access the container with:"
echo "   sudo docker exec -it paddlespeech_dev /bin/bash"
echo -e "\nTo verify GPU support:"
echo "   sudo docker exec paddlespeech_dev python -c \"import paddle; print(paddle.is_compiled_with_cuda())\""
