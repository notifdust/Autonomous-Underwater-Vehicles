# Modular AUV Autonomy Stack (ROS 2 + Gazebo)

This project implements a **complete, realistic autonomy stack** for an Autonomous Underwater Vehicle (AUV) using **ROS 2**, **Gazebo**, and **uuv_simulator** plugins.  
It is designed as a **reference architecture** for underwater robotics, showing how real AUVs integrate perception, control, sensor fusion, simulation, and mission logic inside a fully modular system.

The codebase is structured as a ROS 2 Python package and provides everything needed to run a fully autonomous underwater robot in simulation.  
You can extend it for academic research, course projects, or prototype real AUV software.

---

## What This Project Is

This repository offers a **ready-to-use AUV autonomy framework**, demonstrating:

- How to model a robot with underwater physics.
- How to fuse noisy sensor measurements into a stable state estimate.
- How to design a real-time MPC controller.
- How to allocate thruster forces from 6-DOF wrench commands.
- How to create a mission-level behavior system.
- How all components connect in a “sense → think → act” loop.

It gives you a **solid starting point** for building real underwater robotics systems.

---

## Features

### Simulation
- High-fidelity underwater world in Gazebo.
- Complete `.sdf` model of the AUV with:
  - sensors (IMU, magnetometer, pressure, sonar),
  - thrusters with custom plugin interfaces,
  - hydrodynamic models (added mass, drag, buoyancy).

### Autonomy Stack
- **Mission Planner**  
  State machine issuing depth and heading goals.

- **MPC Controller (6-DOF)**  
  Uses CasADi to compute optimal force/torque based on goals and the current AUV state.

- **Thruster Allocation**  
  Converts a wrench vector into n-thruster commands using the Thruster Configuration Matrix (TCM).

- **Sensor Fusion (EKF)**  
  Fuses IMU + magnetometer + depth sensor into `/odometry/filtered`.

- **Obstacle Avoidance**  
  Sonar-driven artificial potential field generating repulsive commands when obstacles appear.

Everything runs as ROS 2 nodes communicating via topics and services.

---

## System Architecture

The system forms a closed-loop cycle:

1. **Mission Planner**  
   Publishes desired depth/heading to `/target/*`.

2. **MPC Controller**  
   Reads state from `/odometry/filtered` and computes the 6-DOF wrench needed to reach the target.

3. **Thruster Allocator**  
   Converts wrench → per-thruster force vector.

4. **Gazebo Simulation**  
   Applies thruster forces → updates physics → produces sensor data.

5. **Sensor Plugins**  
   Publish noisy IMU, pressure, magnetometer, and sonar measurements.

6. **EKF (robot_localization)**  
   Produces filtered odometry.

7. Controller reads new state → loop repeats at high frequency.

This mirrors real AUV software architecture.

---

## Prerequisites

Install required ROS 2 dependencies:

```bash
sudo apt install ros-$ROS_DISTRO-robot-localization
sudo apt install ros-$ROS_DISTRO-uuv-simulator
You also need:

ROS 2 (Humble, Iron, or Jazzy recommended)

Gazebo (matching your ROS 2 distro)

Python 3 + pip

colcon (sudo apt install python3-colcon-common-extensions)

rosdep

## Installation
bash
Copia codice
mkdir -p auv_ws/src
cd auv_ws

git clone https://your-repo-url.git src/auv_stack

cd src/auv_stack
pip install -r requirements.txt
cd ../..

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
## Usage
1. Source the workspace
bash
Copia codice
source install/setup.bash
2. Start simulation (Gazebo + all nodes)
bash
Copia codice
ros2 launch auv_stack start_simulation.launch.py
This:

launches Gazebo,

loads the underwater world,

spawns the AUV model,

starts the mission planner, controller, thruster allocator, obstacle avoidance, and EKF.

3. Visualization (optional)
bash
Copia codice
rviz2
Recommended topics:

/odometry/filtered — EKF fused state

/sonar_scan — obstacle detection

/tf — frame transforms

## Package Structure
auv_stack/

mission_planner.py — high-level decision making

mpc_controller.py — MPC control loop

thruster_allocator.py — wrench → thrusters

obstacle_avoidance.py — sonar-based avoidance

launch/

start_simulation.launch.py — full system startup

sdf/

auv.sdf — robot, sensors, hydrodynamics, thrusters

worlds/

underwater.world — Gazebo environment

config/

ekf.yaml — EKF configuration

package.xml, setup.py, requirements.txt

## How to Customize the AUV
1. Thruster Layout (critical)
Update thruster positions/orientations in sdf/auv.sdf.

Update the Thruster Configuration Matrix (self.B) in thruster_allocator.py.

If they don’t match → unstable or incorrect motion.

2. Hydrodynamic Parameters
In sdf/auv.sdf, tune:

added_mass

linear_damping

quadratic_damping

buoyancy

inertia tensor

overall mass

These parameters determine how the AUV behaves in water.

3. MPC Tuning
Inside mpc_controller.py modify:

Q → how strongly the controller follows the setpoint

R → how aggressively it uses thrusters

prediction horizon

control horizon

4. EKF Tuning
Edit config/ekf.yaml:

modify covariances,

enable/disable sensors,

adjust process noise.

## What You Can Build From This Template
This stack is flexible and can support:

AUV navigation experiments

Reinforcement learning with realistic physics

Multi-AUV cooperative simulations

Obstacle avoidance benchmarking

Thruster fault tolerance algorithms

MPC and nonlinear control design

High-level mission scripts for underwater inspection