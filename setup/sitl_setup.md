# SITL on Linux

* https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html
* https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/Installing_Ardupilot.md

## Setting up the Build Environment

### Get git
```
sudo apt-get update
sudo apt-get install git
```

### Clone ArduPilot repository
```
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
```
## Install packages

### python packages
```
sudo apt install python3-matplotlib python3-serial python-wxgtk3.0 python-wxtools python3-lxml python3-scipy python3-opencv ccache gawk python3-pip python3-pexpect
```
### MAVLink
```
sudo pip install future pymavlink MAVProxy
```
## Add directories to env path
```
gedit ~/.bashrc
#Add these lines to end of ~/.bashrc (the file open in the text editor):
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH
#Save and close the text editor.
source ~/.bashrc
```
## Open ardupilot dir and run
```
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
```
### Reload the path (log-out and log-in to make permanent):
```
. ~/.profile
```
```
#Compile in ardupilot dir before launch
cd ardupilot
export PATH=$PATH: TARGET_DIR/gcc-arm-none-eabi-10-2020-q4-major/bin
```
```
#Run SITL (Software In The Loop) once to set params:
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
```
# Work Plan

## Manual Flight
```
cd ~/ardupilot/ArduCopter
sim_vehicle.py --map --console
```

## Updating MAVProxy and pymavlink
```
pip install --upgrade pymavlink MAVProxy --user
```
2. right click inside the map once loaded, a window will appear , left click to the region where you want to take your drone and select the option fly to from drop of window 
3. a new dialogue box will appear , here you will need to input the altitude of the drone.
4. have a safe flight .
