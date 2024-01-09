# QGC Stable Build
https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html

# QGC Daily Build
https://docs.qgroundcontrol.com/master/en/releases/daily_builds.html

# Installation
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libqt5gui5 -y
sudo apt install libfuse2 -y

# Logout and login again to enable the change to user permissions.

Download QGroundControl.AppImage
chmod +x ./QGroundControl.AppImage
./QGroundControl.AppImage  (or double click)
