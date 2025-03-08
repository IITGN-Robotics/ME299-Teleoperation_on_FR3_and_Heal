# **HEAL Robot Joint Control and Teleoperation**  

## **Overview**  
We implemented joint command control and keyboard-based teleoperation for the HEAL robot at IITGN. The project consists of multiple scripts that handle different aspects of robot control.  

## **Joint Command Control**  
- The script `joint_comnds.py` is used to send joint commands to the robot using the joint controller.  

## **Keyboard Teleoperation**  
We used keyboard inputs to teleoperate the robot, implementing two different approaches:  
1. **`single_key.py`** – Supports only one key press at a time.  
2. **`multikey.py`** – Allows multiple key presses simultaneously, enabling diagonal movement.  

### **Execution Guide**  

This guide provides step-by-step instructions to execute the robot and run the required control scripts. Follow these steps carefully to ensure smooth operation.  

---

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

## **Step 3: Run the Desired Control Script**  

After setting up the controller, open **another terminal** and follow these steps to execute one of the control scripts.  

### **For Joint Command Control (`joint_comnds.py`)**  

1. **Navigate to the directory where the script is saved:**  
   ```bash
   cd path/to/joint_comnds.py
   ```
2. **Verify the file is present using:**  
   ```bash
   ls
   ```
   If the filename appears in the output, proceed to the next step.  

3. **Run the script:**  
   ```bash
   run joint_comnds.py
   ```
---

### **For Single-Key Teleoperation (`single_key.py`)**  

1. **Navigate to the directory where `single_key.py` is stored:**  
   ```bash
   cd path/to/single_key.py
   ```
2. **Verify the file is present using:**  
   ```bash
   ls
   ```
3. **Run the script:**  
   ```bash
   run single_key.py
   ```

---

### **For Multi-Key Teleoperation (`multikey.py`)**  

1. **Navigate to the directory where `multikey.py` is saved:**  
   ```bash
   cd path/to/multikey.py
   ```
2. **Verify the file is present using:**  
   ```bash
   ls
   ```
3. **Run the script:**  
   ```bash
   run multikey.py
   ```
\[Instead of executing these files on your linux terminals you can directly run your files on VS studio code itself (it's one and the same thing).\]
