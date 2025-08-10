# 7-DOF-Robotic-Arm-Simulation-with-Null-Space-Control-MATLAB

This project simulates a **7-DOF robotic arm** using Denavit–Hartenberg (DH) parameters and the **Peter Corke Robotics Toolbox** for MATLAB.  
It implements joint-space control with **pseudoinverse-based inverse kinematics** and **null-space projection** for a secondary task, while also checking and respecting joint limits.  
Optionally, it can connect to an Arduino to control real servo motors.

---

## Features
- **7-DOF DH kinematic model** using the standard DH convention.
- Forward kinematics with `fkine`.
- Jacobian computation with `jacobe`.
- Null-space control for secondary objectives.
- Joint limit handling using `robot.islimit`.
- Real-time animation with joint trajectory trail.
- Optional **Arduino + servo control block**.

---

## Requirements
- **MATLAB R2020a** or newer (recommended).
- [Peter Corke Robotics Toolbox for MATLAB](https://petercorke.com/toolboxes/robotics-toolbox/) installed and added to MATLAB path.
- *(Optional)* MATLAB Support Package for Arduino Hardware (if using servos).

---

## File Structure
- **`ROS_7DOF_Nullspace_Control.m`** — main simulation script.
  - Self-contained (includes helper function `skew3`).
  - Optional Arduino/servo control block (commented out by default).

---

## Usage
1. Make sure the **Robotics Toolbox** is installed and on the MATLAB path.
2. Open `ROS_7DOF_Nullspace_Control.m` in MATLAB.
3. *(Optional)* If you want to control real servos:
   - Uncomment the Arduino section at the top of the script.
   - Set the correct COM port.
4. Run the script.  
   An animated 3D simulation of the robot will be displayed with a motion trail.

---

## Adjustable Parameters
- **DH Parameters**: Edit in the “DH Parameters” section.
- **Joint Limits**: Change `qlim_deg` (degrees).
- **Simulation Time Step**: Change `dt`.
- **Simulation Duration**: Change `total_time`.
- **Initial Joint Configuration**: Change `q` (in radians).
- **Secondary Task Target**: Change `Pc` and `vu`.

---

## Notes
- The script uses the **body Jacobian** (`jacobe`).  
  If you need the spatial Jacobian, replace `jacobe` with `jacob0` and adjust the math accordingly.
- When using Arduino, you must map the joint angles `q` to the `[0, 1]` range required by `writePosition` for servo motors (not included in this script).

---

## License
MIT License.
