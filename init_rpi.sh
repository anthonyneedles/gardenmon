#!/bin/sh

# Exit script on error.
set -e

if [ ! -f ./gardenmon.py ] ; then
	echo "ERROR: Must run in gardenmon directory" 1>&2
	exit 1
fi

# Update apt as normal.
sudo apt update
sudo apt -y update

# Gotta have vim.
sudo apt install vim -y

# Setup git.
git config --global core.editor "vim"
git config --global credential.helper store

# Enable serial port and enable interactive login.
sudo raspi-config nonint do_serial 0

# Install python3 and smbus library for I2C.
sudo apt install python3 -y
sudo apt install python3-smbus -y

# Install mysql-connector-python
pip install mysql-connector-python

# Enable I2C.
sudo raspi-config nonint do_i2c 0

# Enable 1wire.
sudo raspi-config nonint do_onewire 0

# Create gardenmon service and enable to start after reboot.
sudo sh -c "sed -e 's?\${GARDENMON_PATH}?`pwd`?' gardenmon.service.template > /etc/systemd/system/gardenmon.service"
sudo systemctl enable gardenmon

echo "You should reboot now..."
