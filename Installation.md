<div align="center">
<h1>
ROS2 Humble Installation 
</h1>
</div>
<div align="center">
<img src="https://docs.ros.org/en/humble/_static/humble-small.png" alt="Example Image" width="200"/>
</div>

## ROS2
Robot Operating System (ROS) is a set of software libraries and tools for building robot applications. From drivers and state-of-the-art algorithms to powerful developer tools, ROS has the open source tools you need for your next robotics project.

Since ROS was started in 2007, a lot has changed in the robotics and ROS community. The goal of the ROS 2 project is to adapt to these changes, leveraging what is great about ROS 1 and improving what isn’t.

First thing after installing a fresh Ubuntu is upgrading your system to the latest one, to do that, open a terminal (press Ctrl+Alt+t) on your keyboard and copy the below 👇 commands carefully and press enter to execute a command.

```sh
sudo apt upgrade
```
## Installation Instructions
**(You can refer to the [official installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) from ROS documentation.)**

Here the [distribution](https://docs.ros.org/en/rolling/Releases.html) compatible with Ubuntu 22.04 is the [ROS Humble Hawksbill](http://docs.ros.org/en/humble/). Follow the steps below to install it.
### 1. Set locale
Make sure you have a locale which supports UTF-8.
If you are in a minimal environment (such as a docker container), the locale may be something minimal like POSIX.
We test with the following settings. However, it should be fine if you’re using a different UTF-8 supported locale.
```sh
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

 locale  # verify settings
```
### 2. Setup Sources
You will need to add the ROS 2 apt repository to your system.(More about **curl** [here](https://curl.se/))
```sh
sudo apt install software-properties-common
sudo add-apt-repository universe
```
Now add the ROS 2 GPG key with apt.
```sh
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
Then add the repository to your sources list.
```sh
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
### 3. Install ROS 2 packages
Update your apt repository caches after setting up the repositories.
```sh
sudo apt update
```
ROS 2 packages are built on frequently updated Ubuntu systems. It is always recommended that you ensure your system is up to date before installing new packages.
```sh
sudo apt upgrade
```
**ROS Desktop Install**
* **Desktop Install (Recommended):** ROS, RViz, demos, tutorials. (Make sure you install Desktop and not ROS-Base or Development tools, as we will be using RViz and other softwares/packages which come pre-installed in Desktop installation.)
```sh
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
```
### 4. Environment setup
To Automatically add ROS environment variables to your bash session every time a new shell terminal/bash is launched, enter the following commands (this step is similar as adding environmental variable in windows)

(Find more about bash [here](https://www.gnu.org/software/bash/manual/html_node/index.html#SEC_Contents))
```sh
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
```
* Sourcing bashrc ensures to use updated bashrc using the following command, or it can be done by closing all terminals and re-opening a new terminal.
```sh
source ~/.bashrc
```
### Try some examples
#### Talker-listener
If you installed **ros-humble-desktop** above you can try some examples.

In one terminal, source the setup file and then run a C++ talker:
```sh
ros2 run demo_nodes_cpp talker
```
In another terminal source the setup file and then run a Python listener:
```sh
 ros2 run demo_nodes_py listener
```
You should see the talker saying that it’s Publishing messages and the listener saying I heard those messages. This verifies both the C++ and Python APIs are working properly. 🥳

**Hooray! This confirms that ROS Humble has been installed successfully in your system and is running perfectly.**

## Gazebo Installation <img src="https://avatars.githubusercontent.com/u/1743799?s=200&v=4" alt="Example Image" width="50"/>
To install gazebo please copy the below👇 command
```
sudo apt install ros-humble-gazebo-ros
sudo apt install ros-humble-gazebo-plugins
```
* To source the installed gazebo file
```sh
echo "source /usr/share/gazebo-11/setup.bash" >> ~/.bashrc
source ~/.bashrc      # source bashrc as we have made changes
```