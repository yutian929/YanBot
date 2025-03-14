#!/bin/bash

# Function to check command success
check_success() {
    if [ $? -ne 0 ]; then
        echo "Error occurred in the previous command. Exiting."
        exit 1
    fi
}

apt update
apt install sudo -y
check_success
apt install iputils-ping -y
check_success

pip install flask requests
check_success

python -c "import nltk; nltk.find('.')"
echo "\nNext, clone nltk_data, and move to /root/nltk/"

if [ -d "nltk_data" ]; then
    echo "nltk_data directory already exists. Skipping clone."
else
    echo "Cloning nltk_data..."
    git clone https://github.com/yutian929/YanBot-nltk_data.git nltk_data
    check_success
fi

sudo cp -r ./nltk_data/packages/ /root/nltk_data
check_success
rm -rf ./nltk_data/
check_success
