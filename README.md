# **TurtleBot3 Obstacle Avoidance using Docker and ROS2 Humble**

## **Overview**
This repository provides the procedure to execute a simple LiDAR-based obstacle-avoiding TurtleBot3 using ROS2 Humble.

The algorithm runs within a Docker image, ensuring scalability and efficiency. This approach allows seamless execution on any Ubuntu or Windows system (via WSL).

---

## **Installation and Execution Steps**

### **1. Setup and Build the Docker Image**
1. Ensure **Docker** is installed on your system.
2. Clone the repository using the following command:
   ```bash
   git clone https://github.com/HKyatham/Turtlebot3_Obstacle_Avoidance_with_Docker.git
   ```
3. Navigate to the cloned repository:
   ```bash
   cd Turtlebot3-Obstacle-Avoidance-ROS2
   ```
4. Build the Docker image using the following command:
   ```bash
   docker build . -t <name>
   ```
   - This command uses the `Dockerfile` in the directory to build an image with the specified `<name>`.

5. Verify if the Docker image was successfully created:
   ```bash
   docker images
   ```

6. Run the Docker container using the following command:
   ```bash
   docker run -it --env="DISPLAY=${DISPLAY}" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" <name>
   ```
   - This command triggers the ROS2 launch file inside the container.
   - Replace `<name>` with the name of the container specified earlier.

---

## **Running Only the TurtleBot3 with Manual Control (Teleoperation)**

If you want to run only the TurtleBot3 simulation and control it manually using teleoperation, follow these steps:

### **1. Modify the Dockerfile**
- Edit the `Dockerfile` and **remove** the following `ENTRYPOINT` line:
  ```bash
  ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && \
      source /ros_ws/install/setup.bash && \
      export TURTLEBOT3_MODEL=waffle && \
      export DISPLAY=:0 && \
      ros2 launch enpm_turtlebot3_project new_world.launch.py"]
  ```

### **2. Remove Existing Docker Images (If Any)**
- Check if a Docker image with the same name already exists:
  ```bash
  docker images
  ```
- If you want to remove an existing image:
  ```bash
  docker rmi -f <name>:latest
  ```

### **3. Rebuild the Docker Image**
- Rebuild the image after modifying the `Dockerfile`:
  ```bash
  docker build . -t <name>
  ```

### **4. Run the Docker Container**
- Execute the following command to run the container:
  ```bash
  docker run -it --env="DISPLAY=${DISPLAY}" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" <name>
  ```

### **5. Access the Running Docker Container**
- Get the Docker container ID:
  ```bash
  docker ps
  ```
- Open a new terminal inside the running container:
  ```bash
  docker exec -it <container_id> bash
  ```

### **6. Launch the Teleoperation Node**
- Run the following ROS2 command inside the container to control the TurtleBot3 manually:
  ```bash
  ros2 run enpm_turtlebot3_project turtlebot_teleop.py
  ```

---

## **References**
1. **Gazebo Simulation Tutorials**: [TurtleBot3 Simulation Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
2. **TurtleBot3 Teleoperation**: [GitHub Repository](https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble-devel/turtlebot3_teleop)
