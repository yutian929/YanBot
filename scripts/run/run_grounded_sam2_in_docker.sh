#!/bin/bash

# Function to check command success
check_success() {
    if [ $? -ne 0 ]; then
        echo "Error occurred in the previous command. Exiting."
        exit 1
    fi
}

cd /home/appuser/Grounded-SAM-2/
python gsa_main_in_docker.py --text-prompt "iphone. keyboard. mouse. Water cup." --box-threshold "0.5" --text-threshold "0.4" --auto-save
check_success
