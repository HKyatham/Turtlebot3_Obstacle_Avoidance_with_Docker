# **TurtleBot3 Obstacle Avoidance Using Docker and ROS2 Humble**  

## **Overview**  
This repository provides a step-by-step guide to executing a **LiDAR-based obstacle avoidance** algorithm for the TurtleBot3 using **ROS2 Humble** within a **Docker environment**.  

Running ROS2 inside Docker ensures **scalability, portability, and ease of deployment**, allowing seamless execution on **Ubuntu** and **Windows (via WSL)** without additional system configuration.  

---

## **Installation and Execution**  

### **1. Setting Up and Building the Docker Image**  
1. **Ensure Docker is installed** on your system. If not, install it from [Docker’s official website](https://www.docker.com/get-started).  
2. **Clone this repository** by running the following command:  
   ```bash
   git clone https://github.com/HKyatham/Turtlebot3_Obstacle_Avoidance_with_Docker.git
   ```
3. **Navigate to the cloned repository**:  
   ```bash
   cd Turtlebot3_Obstacle_Avoidance_with_Docker
   ```
4. **Build the Docker image** using the command below:  
   ```bash
   docker build . -t <image_name>
   ```
   - Replace `<image_name>` with a desired name for the Docker image.  
   - The **Dockerfile** in the repository will automatically set up the ROS2 environment and dependencies.  

5. **Verify the Docker image creation**:  
   ```bash
   docker images
   ```
   This will list all available Docker images on your system.  

6. **Run the Docker container**:  
   ```bash
   docker run -it --env="DISPLAY=${DISPLAY}" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" <image_name>
   ```
   - This command runs the ROS2 launch file inside the Docker container.  
   - Replace `<image_name>` with the name specified earlier.  

---

## **Running the TurtleBot3 with Manual Control (Teleoperation)**  

If you want to control the TurtleBot3 **manually** instead of using autonomous obstacle avoidance, follow these steps:  

### **1. Modify the Dockerfile**  
- Open the **Dockerfile** and remove the following `ENTRYPOINT` line:  
  ```bash
  ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && \
      source /ros_ws/install/setup.bash && \
      export TURTLEBOT3_MODEL=waffle && \
      export DISPLAY=:0 && \
      ros2 launch enpm_turtlebot3_project new_world.launch.py"]
  ```

### **2. Remove Existing Docker Images (If Needed)**  
- List existing Docker images:  
  ```bash
  docker images
  ```
- If an image with the same name already exists and needs to be removed:  
  ```bash
  docker rmi -f <image_name>:latest
  ```

### **3. Rebuild the Docker Image**  
- Rebuild the image after modifying the **Dockerfile**:  
  ```bash
  docker build . -t <image_name>
  ```

### **4. Run the Docker Container**  
- Start the Docker container:  
  ```bash
  docker run -it --env="DISPLAY=${DISPLAY}" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" <image_name>
  ```

### **5. Access the Running Docker Container**  
- Find the running container ID:  
  ```bash
  docker ps
  ```
- Open a new terminal inside the running container:  
  ```bash
  docker exec -it <container_id> bash
  ```

### **6. Launch the Teleoperation Node**  
- Run the following ROS2 command inside the container to manually control the TurtleBot3:  
  ```bash
  ros2 run enpm_turtlebot3_project turtlebot_teleop.py
  ```

---

## **Running the Autonomous Behaviors**  

### **1. Running the Tunable Parameter Behavior**  
This behavior allows real-time control of the robot's velocity using keyboard inputs. Below commands have to be executed after executing steps 1~4 in manual operation. 

- **Launch the simulation**:  
  ```bash
  ros2 launch enpm_turtlebot3_project new_world.launch.py
  ```
- **In a new terminal, start the velocity publisher node**:  
  ```bash
  ros2 run enpm_turtlebot3_project lin_vel_publisher
  ```
- Press **‘w’** to increase speed and **‘s’** to decrease speed.

---

### **2. Running the A\* Path Planning Behavior**  
This behavior enables path planning using the **A\* algorithm**, guiding the TurtleBot3 to navigate through a maze. Below commands have to be executed after executing steps 1~4 in manual operation.

- **Launch the simulation**:  
  ```bash
  ros2 launch enpm_turtlebot3_project astar_maze_navigation.launch.py
  ```
- **In a new terminal, start the A\* navigation node**:  
  ```bash
  ros2 run enpm_turtlebot3_project control.py
  ```

---

## **References**  
1. **Gazebo Simulation Tutorials**: [TurtleBot3 Simulation Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)  
2. **TurtleBot3 Teleoperation**: [Official GitHub Repository](https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble-devel/turtlebot3_teleop)  

---

This guide ensures a smooth setup and execution of **TurtleBot3 obstacle avoidance and navigation behaviors** using **Docker and ROS2 Humble**. 🚀