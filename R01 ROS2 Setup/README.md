# ROS2 Installation and Setup

## Windows

### Installing Oracle VirtualBox and Ubuntu 22.04

1. Download Oracle VirtualBox from the official website and install it on your Windows machine.
2. Download the Ubuntu 22.04 LTS ISO from the official Ubuntu website.
3. Open VirtualBox and create a new virtual machine:
   - Name: Ubuntu 22.04
   - Type: Linux
   - Version: Ubuntu (64-bit)
   - Allocate at least 4GB RAM and 25GB hard disk space
4. Mount the Ubuntu ISO to the virtual machine and start it.
5. Follow the Ubuntu installation prompts to complete the setup.

### Installing ROS2 Humble and Tools

1. Open a terminal in the Ubuntu VM.
2. Set up the ROS2 repository:
   ```bash
   sudo apt update && sudo apt install curl -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```
3. Install ROS2 Humble:
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   ```
4. Install Visual Studio Code:
   - Download the .deb file from the VS Code website.
   - Install using:
     ```bash
     sudo dpkg -i <path-to-downloaded-file>.deb
     sudo apt-get install -f
     ```
5. Install Colcon:
   ```bash
   sudo apt install python3-colcon-common-extensions
   ```

### Creating a Workspace

1. Create and build a ROS2 workspace:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   colcon build
   ```
2. Source the workspace:
   ```bash
   source install/setup.bash
   ```
3. Add ROS2 sourcing to `.bashrc`:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

## macOS

For macOS users, UTM is an excellent choice for creating virtual machines, especially on Macs with Apple Silicon (M1, M2, or M3 chips). Here's how to set up UTM and create a virtual machine:

### Installing UTM

1. Download UTM from the official website (https://mac.getutm.app) or the Mac App Store[1][2].
2. If downloaded from the website, open the UTM.dmg file and drag the UTM icon to the Applications folder[3].
3. Launch UTM from the Applications folder or Launchpad[1].

### Creating a Virtual Machine

1. Open UTM and click on the "Create a New Virtual Machine" button[5].
2. Choose "Virtualize" for macOS 12+ or "Other" for earlier versions[4].
3. For macOS 12+, you'll need an Apple recovery IPSW file. UTM can download the latest version automatically, or you can provide your own[4][5].
4. Follow the on-screen prompts to configure your virtual machine, including allocating RAM and storage[5].
5. Name your virtual machine and click "Save"[5].
6. Start the virtual machine by clicking the play button[5].

UTM offers a user-friendly interface and supports a wide range of operating systems, making it a versatile choice for virtualization on macOS, especially for Apple Silicon Macs[1][4].

### Installing ROS2 Humble and Tools

1. Open a terminal in the Ubuntu VM.
2. Set up the ROS2 repository (same commands as in the Windows section).
3. Install ROS2 Humble (same commands as in the Windows section).
4. Install Visual Studio Code:
   - Download the .deb file from the VS Code website.
   - Install using the same commands as in the Windows section.
5. Install Colcon (same command as in the Windows section).

### Creating a Workspace

Follow the same steps as in the Windows section to create and set up your ROS2 workspace.

## Additional Notes

- For both Windows and macOS, ensure your virtual machine has nested virtualization enabled for better performance.
- Consider allocating more resources (RAM, CPU cores) to the virtual machine if your host system allows it.
- Keep your Ubuntu system and ROS2 installation updated regularly:
  ```bash
  sudo apt update && sudo apt upgrade
  ```
- If you encounter any issues with USB or network connectivity in the virtual machine, consult the documentation for your virtualization software (VirtualBox, VMware Fusion, or Parallels).


Sources
[1] How to Install and use UTM on Mac - YouTube https://www.youtube.com/watch?v=i8yUvohhAXw
[2] macOS - UTM Documentation https://docs.getutm.app/installation/macos/
[3] ArcGIS Pro Installation MacOS (Apple Silicon) - GitHub Pages https://umtgis.github.io/guides/install/mac_apple_silicon.html
[4] How to use UTM to run almost any version of macOS - AppleInsider https://appleinsider.com/inside/macos/tips/how-to-use-utm-to-run-almost-any-version-of-macos----even-very-old-ones
[5] Install macOS Sequoia Beta in a Virtual Machine on an M1, M2, or ... https://www.intego.com/mac-security-blog/install-macos-ventura-beta-in-a-virtual-machine-on-an-m1-or-m2-mac-with-utm/
