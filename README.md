## **Step 1: Start the Robot and Controller**  

1. **Power on the robot and its controller.**  
2. **Connect to the robot's controller from your local PC using SSH:**  
   ```bash
   ssh cobot@192.168.1.25
   ```
3. **Enter the password when prompted.**  

   This establishes a connection with the robot's controller.  

4. **Navigate to the controller's execution directory:**  
   ```bash
   cd tests/build/
   ```
   This moves into the required directory for running the robot's server.  

5. **Start the Heal server:**  
   ```bash
   sudo ./heal_server
   ```
   This launches the server that manages the robot’s hardware and communication.  

6. **Home position:**
   To bring the HEAL robot to its hime position you can run the following code on linux
   ```bash
   sudo ./base_rigid
   ```
   This launches the server that manages the robot’s hardware and communication. Ensure you execute this code before powering off the robot.
---

## **Step 2: Launch the Robot Controller**  

1. Open a **new terminal** on your PC.  
2. Navigate to the directory where the controller is stored:  
   ```bash
   cd Debojit_WS/Addverb_Heal_and_Syncro_Hardware
   ```
3. **Source the workspace:**  
   ```bash
   source devel/setup.bash
   ```
   This ensures all environment variables are properly set.  
4. **Launch the controller using ROS:**  
   ```bash
   roslaunch addverb_cobot_control bringup.launch
   ```
5. **Important: Ensure the correct control mode is set in the configuration file.**  
   - Open the configuration file:  
     ```bash
     nano addverb_cobot_control/config/default_control.yaml
     ```
   - Ensure the following line is correctly set:  
     ```yaml
     ros_control_mode: velocity
     ```
   - If set to **effort**, update it to **velocity** to prevent errors.  
   - Make the necessary changes in the **bringup.launch** file if required.  

---
## **Step 2: Running the script files**  

This repository contains Python scripts to teleoperate the **Heal Robot** in two modes:  
1. **Joint Space Teleoperation** using `joint_command.py`  
2. **Cartesian Space Teleoperation** using `cartesian_teleop.py`  

---

### **1. Run Joint Space Teleoperation (`joint_command.py`)**  

To control the robot directly in **joint space**, follow these steps:  

**Step 1:** Navigate to the workspace directory:  
```bash
cd ~/ME299-PC/src
```  

**Step 2:** Source your workspace:  
```bash
source devel/setup.bash
```  

**Step 3:** Run the joint space teleoperation script:  
```bash
rosrun heal_teleop joint_command.py
```  

---

### **2. Run Cartesian Space Teleoperation (`cartesian_teleop.py`)**  

To control the robot in **Cartesian space (X, Y, Z)**, follow these steps:  

**Step 1:** Navigate to the workspace directory:  
```bash
cd ~/ME299-PC/src
```  

**Step 2:** Source your workspace:  
```bash
source devel/setup.bash
```  

**Step 3:** Run the Cartesian space teleoperation script:  
```bash
rosrun heal_teleop cartesian_teleop.py
```  

---
\[Instead of executing these files on your linux terminals you can directly run your files on VS studio code itself (it's one and the same thing).\]
