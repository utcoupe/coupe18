#!/bin/bash

### The goal of this script is to install all UTCoupe specific packages to have a working setup.
### This script is called automatically when you run a npm install.

function green_echo() {
	echo -e "\033[32m$1\033[0m"
}

function red_echo() {
	echo -e "\033[31m$1\033[0m"
}

# Globals
ARCH=$(uname -m)

### Install the linux packages
function install_apt() {
	green_echo "Install missing packages..."
	sudo apt-get install git build-essential python cmake libboost-dev libsdl1.2-dev gcc-avr avrdude avr-libc

	# Check if it's a PC or a raspi
	if [ "$ARCH" = "x86_64" ]; then
		green_echo "x86 architecture detected."
		sudo apt-get install nodejs npm nodejs-legacy linux-headers-$(uname -r)
	elif [ "$ARCH" = "armv7l" ]; then
		green_echo "Raspberry Pi 3 system detected, remove previous npm installation to setup the used version."
		sudo apt-get install raspberrypi-kernel-headers
		sudo apt-get remove npm nodejs nodejs-legacy
		curl -sL https://deb.nodesource.com/setup_4.x | sudo -E bash -
		sudo npm install npm@3.5.2 -g
	elif [ "$ARCH" = "armv6l" ]; then
		sudo apt-get install raspberrypi-kernel-headers
		sudo apt-get remove npm nodejs nodejs-legacy
		wget https://nodejs.org/dist/v4.8.1/node-v4.8.1-linux-armv6l.tar.gz
		tar -xvf node-v4.8.1-linux-armv6l.tar.gz
		cd node-v4.8.1-linux-armv6l
		sudo cp -R * /usr/local/
		cd ..
		sudo rm -rf node-v4.8.1-linux-armv6l
	else
		red_echo "ARCH of system not corresponding to a know arch... ($ARCH)"
	fi
}


### Setup the variable environment to taget the UTCoupe main folder
function env_setup() {
	# Add the UTCOUPE_WORKSPACE env variable, default consider as bash shell
	if [ -z "$UTCOUPE_WORKSPACE" ]; then
		green_echo "Env variable is not set."
		if [ "$SHELL" = "/bin/zsh" ]; then
			echo "export UTCOUPE_WORKSPACE=$PWD" >> $HOME/.zshrc

			printf "Warning :\n"
			printf "Please \"source ~/.zshrc\" and run again this script if necessary\n"
			exit 1
		else
			echo "export UTCOUPE_WORKSPACE=$PWD" >> $HOME/.bashrc
            		source $HOME/.bashrc
		fi
	fi
	# Add a file where to find UTCOUPE_WORKSPACE for node launched at startup
	if [ ! -f "/etc/default/utcoupe" ]; then
		sudo touch "/etc/default/utcoupe"
		sudo sh -c "echo 'UTCOUPE_WORKSPACE=$PWD' >> /etc/default/utcoupe"
	fi
	# Add the current user to the dialout group (to r/w in /dev files)
	if ! id -Gn $USER | grep -qw "dialout"; then
	        sudo usermod -a -G dialout $USER
	fi
	# Setup GPIO + add the current user to the gpio group (to r/w in /dev files)
	if ! id -Gn $USER | grep -qw "gpio"; then
			sudo chgrp -R gpio /sys/class/gpio
			sudo chmod -R g+rw /sys/class/gpio
	        sudo usermod -a -G gpio $USER
	fi
	# Create the utcoupe folder where log files are stored
	if [ ! -d "/var/log/utcoupe" ]; then
		sudo mkdir /var/log/utcoupe
	fi
	# Untar all libraries
	for f in $UTCOUPE_WORKSPACE/libs/*.tar; do tar -C $UTCOUPE_WORKSPACE/libs xzf $f; done
	# "Install" Arduino libs
	sudo ln -s $UTCOUPE_WORKSPACE/libs/arduino-1.0 /opt/
	# Change the ownership of the utcoupe log folder
	sudo chown $USER:$USER /var/log/utcoupe
	# Install the hokuyo automatic startup script (only for raspberry pi zero)
	if [ ! -f "/etc/init.d/utcoupe_hokuyo.sh" ] && [ "$ARCH" = "armv6l" ]; then
		sudo install $UTCOUPE_WORKSPACE/scripts/utcoupe_hokuyo.sh /etc/init.d/
		sudo update-rc.d utcoupe_hokuyo.sh defaults 99
	fi
}

### Compile and install the UTCoupe libraries

# URG library for the hokuyo
function compile_urg() {
	cd $UTCOUPE_WORKSPACE/libs/urg-0.8.18
	./configure && make && sudo make install
}

# Archer driver for 5 GHz wifi
function compile_archer() {
	cd $UTCOUPE_WORKSPACE/libs/Archer_T1U_V1_150909/Driver
	sudo make && sudo make install
	#TODO add the ra0 interface in configuration files
}

### Then install the UTCoupe softwares

# The pathfinding
function compile_pathfinding() {
	cd $UTCOUPE_WORKSPACE/pathfinding
	./make.sh
}

# The hokuyo
function compile_hokuyo() {
	cd $UTCOUPE_WORKSPACE/hokuyo
	./make.sh
}

function launch_script() {

	env_setup
	
	printf "Install apt missing packets ? [Y/n]?"
	read answer
	if [ "$answer" = "" ] || [ "$answer" = "y" ] || [ "$answer" = "Y" ]; then
		install_apt
	fi
	
	printf "Compile archer library (mandatory for 5 GHz usb wifi key) ? [Y/n]?"
	read answer
	if [ "$answer" = "" ] || [ "$answer" = "y" ] || [ "$answer" = "Y" ]; then
		compile_archer
	fi
	
	printf "Compile pathfinding ? [Y/n]?"
	read answer
	if [ "$answer" = "" ] || [ "$answer" = "y" ] || [ "$answer" = "Y" ]; then
		compile_pathfinding
	fi
	
	printf "Compiled hokuyo (+ urg mandatory library) ? [Y/n]?"
	read answer
	if [ "$answer" = "" ] || [ "$answer" = "y" ] || [ "$answer" = "Y" ]; then
		compile_urg
		compile_hokuyo
	fi
}

# Verify that the script is launched from the right place
if [ ! "${PWD##*/}" = "coupe17" ]; then
	red_echo "You have to launch this script from UTCoupe main directory : ./script/${0##*/}"
	exit 1
fi

# Ask the user if he wants launch the script
printf "Launch install script ? [Y/n]?"
read answer
if [ "$answer" = "" ] || [ "$answer" = "y" ] || [ "$answer" = "Y" ]; then
	launch_script
fi

