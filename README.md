# AMR Simulation

Simulation of an Autonomous Mobile Robot (AMR) for garage inspection.

---

## Prerequisites

You need **ROS 2 Humble** installed, either on the host system or inside a Docker container.

### Option 1: Install on Host

Follow the official [ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation.html).

### Option 2: Run in Docker (recommended)

A `docker-compose` utility is provided:

```bash
docker compose up
docker exec -it nav2_humble bash
```

---

## Install Dependencies

Update packages:

```bash
apt-get update
```

Install required tools and libraries:

```bash
apt install -y python3-pip
apt install -y alsa-utils
apt install -y python3-colcon-argcomplete
apt install -y ros-humble-joint-state-publisher-*
apt install -y gazebo
apt install -y ros-humble-gazebo-*
```

---

## Build the Workspace

From the workspace root:

```bash
colcon build
```

---

## Prepare Gazebo Models

1. Go to (or create) the Gazebo models directory:

   ```bash
   cd ~/.gazebo/ || mkdir -p ~/.gazebo && cd ~/.gazebo/
   ```

2. Clone and copy required models:

   ```bash
   git clone https://github.com/osrf/gazebo_models.git models
   cp -r ugv_gazebo/models/world models  # provided from the repo
   cp -r ugv_description models          # # provided from the repo
   cd ~/.gazebo/models/
   ```

---

## Start the Simulation

Set the UGV model and launch Gazebo:

```bash
export UGV_MODEL=ugv_rover
ros2 launch ugv_gazebo bringup.launch.py
```

---

