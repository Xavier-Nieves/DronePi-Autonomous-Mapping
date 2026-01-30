# COMPLETE BEGINNER'S GUIDE - Unitree L1 LiDAR Setup

This guide will walk you through everything from scratch, assuming you're new to Docker and ROS.

## 📋 What You'll Need

1. **A computer running Linux** (Ubuntu 20.04 or 22.04 recommended)
   - Windows/Mac users: You'll need to run Linux in a virtual machine or dual boot
2. **Unitree L1 4D LiDAR** connected via USB cable
3. **Internet connection** to download software

---

## STEP 1: Install Docker

Open a terminal (Ctrl+Alt+T on Ubuntu) and run these commands one by one:

```bash
# Update your system
sudo apt-get update
sudo apt-get upgrade -y

# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Add your user to docker group (so you don't need sudo)
sudo usermod -aG docker $USER

# Install Docker Compose
sudo apt-get install docker-compose -y

# IMPORTANT: Log out and log back in for the docker group to take effect
# Or reboot your computer
```

After logging back in, verify Docker is working:
```bash
docker --version
docker-compose --version
```

You should see version numbers printed out.

---

## STEP 2: Download the Project Files

Create a folder for your project and download the files:

```bash
# Go to your home directory
cd ~

# Create a project folder
mkdir unitree_lidar_project
cd unitree_lidar_project

# Download the project files
# (You'll need to copy the Dockerfile.ros1, Dockerfile.ros2, docker-compose.yml, 
#  start.sh, and README.md files into this folder)
```

**How to get the files:**
1. Download all the files I provided to you
2. Copy them into the `~/unitree_lidar_project` folder
3. Make sure these files are in the folder:
   - `Dockerfile.ros1`
   - `Dockerfile.ros2`
   - `docker-compose.yml`
   - `start.sh`
   - `README.md`
   - `.gitignore`

To verify the files are there:
```bash
ls -la
```

You should see all the files listed.

Make the start script executable:
```bash
chmod +x start.sh
```

---

## STEP 3: Connect Your LiDAR

1. **Plug in your Unitree L1 LiDAR** to your computer via USB cable
2. **Find the device name:**

```bash
# Wait 5 seconds after plugging it in, then run:
ls -l /dev/ttyUSB*
```

You should see something like:
```
crw-rw---- 1 root dialout 188, 0 Jan 28 10:30 /dev/ttyUSB0
```

The device is usually `/dev/ttyUSB0` (this is the default in our setup).

**If you see a different name** (like `/dev/ttyUSB1`), you'll need to edit `docker-compose.yml`:
```bash
nano docker-compose.yml
```

Find this line:
```yaml
- /dev/ttyUSB0:/dev/ttyUSB0
```

Change both instances of `ttyUSB0` to your device name (e.g., `ttyUSB1`).

Press `Ctrl+X`, then `Y`, then `Enter` to save and exit.

---

## STEP 4: Build the Docker Image

This will download and set up everything you need. It will take 15-30 minutes the first time.

**Choose ONE of these options:**

### Option A: Interactive Menu (Easiest)
```bash
./start.sh
```

When the menu appears, press `2` and Enter to build the ROS2 Humble image.

### Option B: Direct Command
```bash
docker-compose build unitree-l1-ros2
```

**What's happening:**
- Docker is downloading Ubuntu 22.04
- Installing ROS2 Humble
- Installing all LiDAR software
- Setting everything up automatically

**Go get coffee!** ☕ This takes a while the first time.

When you see "Successfully built" and "Successfully tagged", you're done!

---

## STEP 5: Run the Container

Now let's start the Docker container with all the software inside.

### Enable Display (for visualization)
First, allow Docker to show windows on your screen:
```bash
xhost +local:docker
```

### Start the Container

**Option A: Using the menu**
```bash
./start.sh
```
Press `4` and Enter to run the ROS2 Humble container.

**Option B: Direct command**
```bash
docker-compose run --rm unitree-l1-ros2
```

**You should now see a prompt like:**
```
root@your-computer:/root/ros2_ws#
```

🎉 **You're now INSIDE the Docker container!** This is like a virtual computer with all the LiDAR software ready to go.

---

## STEP 6: Test Your LiDAR

Now let's see if your LiDAR is working!

### Inside the container, run:

```bash
# First, check if the LiDAR device is visible
ls -l /dev/ttyUSB0
```

You should see the device listed. If not, go back to Step 3.

### Start the LiDAR driver:

```bash
# Load the software environment
source /root/ros2_ws/install/setup.bash

# Start the LiDAR
ros2 launch unitree_lidar_ros2 launch.py
```

**What you should see:**
```
[INFO] Unilidar initialization succeed!
[INFO] lidar firmware version = ...
```

The LiDAR should start spinning and collecting data!

---

## STEP 7: Visualize the Data

Let's see what the LiDAR is seeing!

**Open a NEW terminal window** on your host computer (Ctrl+Alt+T), then run:

```bash
# Connect to the running container
docker exec -it unitree_l1_ros2 bash

# Inside the container, load the environment
source /root/ros2_ws/install/setup.bash

# Start the visualization tool
rviz2
```

**RViz2 window should open!** Now set it up:

1. In the left panel, find "Fixed Frame"
2. Change it from "map" to `unilidar_lidar`
3. Click the "Add" button at the bottom left
4. Select "By topic" tab
5. Find `/unilidar/cloud` → PointCloud2 and click "OK"
6. You should see the point cloud visualization!

**Optional - Add IMU data:**
1. Click "Add" again
2. Go to "By topic"
3. Find `/unilidar/imu` → Imu and add it
4. You'll see a small coordinate frame showing the IMU orientation

---

## STEP 8: Stop Everything

When you're done:

### In the terminal running the LiDAR:
Press `Ctrl+C` to stop the LiDAR driver

### In the RViz window:
Close the window

### Exit the container:
```bash
exit
```

Type `exit` in both terminal windows to leave the container.

---

## 🎯 SUMMARY - Daily Use

After the first setup, here's all you need to do each time:

```bash
# 1. Go to your project folder
cd ~/unitree_lidar_project

# 2. Enable display
xhost +local:docker

# 3. Start the container
docker-compose run --rm unitree-l1-ros2

# 4. Inside container - Start LiDAR
source /root/ros2_ws/install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py

# 5. In a NEW terminal - Start visualization
docker exec -it unitree_l1_ros2 bash
source /root/ros2_ws/install/setup.bash
rviz2
```

---

## 🔧 Common Issues & Solutions

### "Permission denied" when accessing /dev/ttyUSB0
```bash
# On your HOST computer (not in Docker), run:
sudo chmod 666 /dev/ttyUSB0
```

### "Cannot connect to Docker daemon"
```bash
# Make sure Docker is running:
sudo systemctl start docker

# Check status:
sudo systemctl status docker
```

### "Device not found"
```bash
# Check if LiDAR is connected:
ls -l /dev/ttyUSB*

# Try unplugging and replugging the USB cable
# Wait 5 seconds and check again
```

### RViz2 won't open / Display error
```bash
# On host computer:
xhost +local:docker

# If still not working, try:
echo $DISPLAY
# Should show something like ":0" or ":1"
```

### Build failed / Out of space
```bash
# Clean up Docker to free space:
docker system prune -a

# Then try building again
```

---

## 🚀 Next Steps - Using Point-LIO SLAM

Once you're comfortable with the basics, you can try the advanced SLAM features:

```bash
# Inside the container

# Terminal 1 - Start Point-LIO SLAM
source /root/ros2_ws/install/setup.bash
ros2 launch point_lio mapping_unilidar_l1.launch.py

# Terminal 2 - Start LiDAR driver
docker exec -it unitree_l1_ros2 bash
source /root/ros2_ws/install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py
```

**Important:** Keep the LiDAR perfectly still for the first 5 seconds when starting Point-LIO!

The SLAM will create a 3D map as you move the LiDAR around. The map is saved automatically to `/root/ros2_ws/PCD/scans.pcd`

---

## 📞 Need Help?

If you get stuck:

1. **Check the error message** - Often it tells you what's wrong
2. **Google the error** - Someone else probably had the same issue
3. **Check the main README.md** - Has more detailed troubleshooting
4. **Ask for help** - Provide the full error message when asking

---

## 🎓 Learning More

Once this is working, you might want to learn:

- **Basic ROS2 concepts** - [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- **Docker basics** - [Docker Get Started](https://docs.docker.com/get-started/)
- **Point cloud processing** - Explore PCL (Point Cloud Library)
- **SLAM concepts** - Read about Point-LIO algorithm

Good luck! 🚀
