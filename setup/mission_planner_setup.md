## Mission Planner on Linux using MONO
https://ardupilot.org/planner/docs/mission-planner-installation.html
https://www.mono-project.com/download/stable/

## Ubuntu 20.04
# Add the Mono repository to your system
sudo apt install ca-certificates gnupg
sudo gpg --homedir /tmp --no-default-keyring --keyring /usr/share/keyrings/mono-official-archive-keyring.gpg --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 3FA7E0328081BFF6A14DA29AA6A19B38D3D831EF
echo "deb [signed-by=/usr/share/keyrings/mono-official-archive-keyring.gpg] https://download.mono-project.com/repo/ubuntu stable-focal main" | sudo tee /etc/apt/sources.list.d/mono-official-stable.list
sudo apt update

## If you encounter this error,
N: Skipping acquire of configured file 'main/binary-i386/Packages' as repository 'https://download.mono-project.com/repo/ubuntu stable-focal InRelease' doesn't support architecture 'i386'

## Solution
# To confirm you are using 64 bit ubuntu with multiarch enabled issue
dpkg --print-foreign-architectures
# Output: i386, Then to remove multi architecture ( only if you have no 32 bit applications )
sudo dpkg --remove-architecture i386

## Install Mono
# mono-devel should be installed to compile code
sudo apt install mono-devel
# or, install everything
sudo apt install mono-complete

## Download Mission_Planner.zip
https://firmware.ardupilot.org/Tools/MissionPlanner/MissionPlanner-latest.zip

## Unzip and Change to that directory and execute
cd MissionPlanner
mono MissionPlanner.exe
