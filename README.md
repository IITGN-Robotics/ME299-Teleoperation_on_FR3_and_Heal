# **HEAL Robot Joint Control and Teleoperation**  

## **Overview**  
We implemented joint command control and keyboard-based teleoperation for the HEAL robot at IITGN. The project consists of multiple scripts that handle different aspects of robot control.  

## **Joint Command Control**  
- The script `joint_comnds.py` is used to send joint commands to the robot using the joint controller.  

## **Keyboard Teleoperation**  
We used keyboard inputs to teleoperate the robot, implementing two different approaches:  
1. **`single_key.py`** – Supports only one key press at a time.  
2. **`multikey.py`** – Allows multiple key presses simultaneously, enabling diagonal movement.  
