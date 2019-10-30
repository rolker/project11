# Setting up the backseat machine mystique for the CW4

The following steps are for installing Ubuntu server 18.04 on an Intel NUC along with the Project11 environment for the L3 ASV CWorker 4.

## VLAN config

The following network setup is from existing mystique setup.

    cobham100,    vid: 100, ip: 192.168.100.112/24
    cw4_internal, vid: 10,  ip: 192.168.10.112/24
    cw4wifi,      vid: 101, ip: 192.168.101.112/24
    payload15,    vid: 15,  ip: 192.168.15.112/24

## Install Ubbuntu Server 18.04

This was accomplished with monitor and keyboard connected to an Intel NUC.

A USB key was setup as a boot disk with the ubuntu-18.04.2-live-server-amd64.iso image. (This does not work on the CCOM network since it requires a connection over ethernet and the NUCs are not configure to get a DHCP address) This image does NOT support installing over wifi.

Discussion about installing Ubuntu Server over wifi: https://askubuntu.com/questions/1032644/ubuntu-server-18-04-setting-wifi-connections-when-install

The following image was installed on the USB key.

http://cdimage.ubuntu.com/releases/18.04.2/release/ubuntu-18.04.2-server-amd64.iso

Boot off the USB key, select install, and follow the prompts to configure the keyboard.

At network detection, let it timeout or cancel the DHCP portion. Select configure network later.

Set machine name to mystique.

Add Field User account with username field.

If asked to unmount partitions, do so. Then select use entire disk when asked how to partition the drive.

Select no automatic updates.

At software selection, check the OpenSSH server for installation

At the prompt, remove USB key and reboot into newly installed system.

## Wifi setup

Extra packages are needed from the USB key. Insert it and mount it.

    mkdir usbkey
    sudo mount /dev/sdb usbkey

Install wpasupplicant and dependency.

    sudo dpkg --install usbkey/pool/main/p/pcsc-lite/libpcsclite1_1.8.23-1_amd64.deb
    sudo dpkg --install usbkey/pool/main/w/wpa/wpasupplicant_2.6-15ubuntu2.1_amd64.deb
    
Create a netplan file for the wifi insterface:

    sudo nano /etc/netplan/wifi.yaml
    
In the file, add an entry for the wifi interface. Note that the interface is made optional so that booting doesn't get delayed by a 2 minute timeout if the wifi network is not avaiable.

    network:
      version: 2
      renderer: networkd
      wifis:
        wlp1s0:
          dhcp4: yes
          optional: yes
          access-points:
            "CCOMGuest":
              password: "**********"
              
Save and apply.

    sudo netplan apply
    
At this point, networking should be available via Wifi and further packages can be installed via the network so the usb key can be unmounted and removed.

    sudo umount usbkey
    rmdir usbkey/

Alternativelty, the above intructions can be adjusted for a wired network to gain initial network conectivity.

## Installing ROS

Follow the instructions on the [Ubuntu install of ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) page, picking the Desktop installation.

## Configuring git.

Setup git user to be Field User.

    git config --global user.name "Field User"
    git config --global user.email field@ccom.unh.edu

## Creating the Project 11 directory structure.

Part of the directory structure is created by hand while others are cloned from git repositories.

    mkdir -p ~/project11/catkin_ws/src
    mkdir -p ~/project11/log/moos
    mkdir -p ~/project11/log/nodes
    mkdir -p ~/project11/executed_missions

To be able to contain all the relevant data inside the project11 directory, ROS' default working directory, ~/.ros, gets moved and replaced with a symlink.

    mv ~/.ros ~/project11/ros
    ln -s ~/project11/ros ~/.ros

If an error occurs complaining that ~/.ros does not exist. Create it instead of moving it: mkdir ~/project11/ros

Create a few simlinks so that ROS nodes can log to the log directory.

    ln -s ~/project11/log/nodes ~/project11/ros/nodes

Initialize the ROS Catkin workspace.

    cd ~/project11/catkin_ws
    catkin_make

Normally, the Project 11 Catkin workspace is the only one used, so we can make it the default workspace when logging in by sourcing the setup script from .bashrc.

    echo "source ~/project11/catkin_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    
Fetch a few more directories from github.

    cd ~/project11
    git clone https://github.com/CCOMJHC/project11_documentation.git documentation
    git clone https://github.com/CCOMJHC/project11_scripts.git scripts
    git clone https://github.com/CCOMJHC/project11_configuration.git config

Clone ROS packages from github:

    cd ~/project11/catkin_ws/src
    git clone https://github.com/CCOMJHC/asv_global.git
    git clone https://github.com/CCOMJHC/cw4_helm.git
    git clone https://github.com/CCOMJHC/project11.git
    git clone https://github.com/CCOMJHC/project11_transformations.git
    git clone https://github.com/CCOMJHC/udp_bridge.git
    git clone https://github.com/CCOMJHC/marine_msgs.git
    git clone https://github.com/CCOMJHC/mission_plan.git
    git clone https://github.com/CCOMJHC/command_bridge.git
    git clone https://github.com/CCOMJHC/mission_manager.git
    git clone https://github.com/CCOMJHC/dubins_curves.git
    git clone https://github.com/CCOMJHC/path_follower.git
    git clone https://github.com/CCOMJHC/kongsberg_em.git
    git clone https://github.com/CCOMJHC/kongsberg_em_control.git
    git clone https://github.com/CCOMJHC/geographic_visualization_msgs.git
    git clone https://github.com/CCOMJHC/hover.git
    git clone https://github.com/CCOMJHC/br24_radar.git
    git clone https://github.com/CCOMJHC/posmv.git
    
    
Clone darknet package, but only build the messages:

    git clone https://github.com/leggedrobotics/darknet_ros.git
    touch darknet_ros/darknet_ros/CATKIN_IGNORE
    
Install additional ROS packages.

    sudo apt install ros-melodic-geographic-msgs
    sudo apt install ros-melodic-geodesy
    sudo apt install ros-melodic-video-stream-opencv
    sudo apt install ros-melodic-pid

Build project11 and test.

    cd ~/project11/catkin_ws
    catkin_make
    source devel/setup.bash
    roslaunch project11 mystique.launch

Verify that all nodes start up properly. If roslaunch is run with mystique not in the asv, expect error from devices that are not avaiable, such as cameras.
    
## Wired network setup

The wired network configuration consisting of VLAN configs for the wired network. Ubuntu server 18.04 uses netplan for network configuration. Copy the config file from the project11 config directory.

    sudo cp ~/project11/config/mystique/mystique_wired_network.yaml /etc/netplan/
    sudo netplan apply

### Host file

TODO: add to /etc/hosts
    
## Time synchronization

Install chrony.

    sudo apt install chrony

TODO: add chrony config.

## Useful tools

Install usefull tools. Some can use X remotely.

    sudo apt install xterm
    sudo apt install gedit
    sudo apt install wireless-tools
    sudo apt install xinit
    sudo apt install fluxbox
    
    
