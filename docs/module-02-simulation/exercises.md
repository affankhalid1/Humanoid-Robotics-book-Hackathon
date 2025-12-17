# Module 2: Simulation Exercises

## Exercise 2.1: Basic Gazebo Environment Setup

### Objective
Set up a basic Gazebo simulation environment and understand the client-server architecture.

### Tasks
1. Install Gazebo and ROS 2 Humble integration packages
2. Launch Gazebo with the empty world
3. Verify both gzserver and gzclient are running
4. Explore the Gazebo GUI interface

### Steps
1. Install required packages:
   ```bash
   sudo apt update
   sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins ros-humble-gazebo-dev
   ```
2. Launch Gazebo:
   ```bash
   gz sim -r empty.sdf
   ```
3. Verify the installation by checking available topics:
   ```bash
   gz topic -l
   ```

### Deliverables
- Screenshot of Gazebo GUI with empty world
- List of available Gazebo topics
- Explanation of client-server architecture in your own words

### Evaluation Criteria
- Gazebo launches successfully
- All required packages are installed
- You can identify client and server components

---

## Exercise 2.2: Create a Simple Robot Model

### Objective
Create a basic robot model using URDF with links, joints, and visual/collision properties.

### Tasks
1. Create a URDF file for a simple wheeled robot
2. Include at least 3 links and 2 joints
3. Define visual and collision properties for each link
4. Add proper inertial properties

### Requirements
- Base link with box geometry
- Two wheel links with cylinder geometry
- Continuous joints connecting wheels to base
- Realistic mass and inertia values

### Steps
1. Create a new URDF file: `simple_robot.urdf`
2. Define the robot structure with proper XML formatting
3. Add visual, collision, and inertial properties
4. Validate the URDF using `check_urdf`

### Deliverables
- Complete URDF file
- URDF validation output showing no errors
- Screenshot of the robot model in Gazebo

### Evaluation Criteria
- URDF validates without errors
- Robot has proper physical properties
- All required elements are present

---

## Exercise 2.3: Gazebo World Creation

### Objective
Create a custom Gazebo world with physics properties and static objects.

### Tasks
1. Create an SDF world file
2. Configure physics engine parameters
3. Add static models (ground plane, obstacles)
4. Include lighting and environmental settings

### Requirements
- Physics engine with custom gravity
- At least 3 static objects
- Proper coordinate systems
- Realistic lighting

### Steps
1. Create world file: `custom_world.sdf`
2. Define physics properties
3. Add models using `<include>` tags or inline definitions
4. Test the world in Gazebo

### Deliverables
- Custom world SDF file
- Screenshot of the world in Gazebo
- Explanation of physics parameters chosen

### Evaluation Criteria
- World loads without errors
- Physics parameters are appropriately set
- Objects are properly positioned

---

## Exercise 2.4: Sensor Integration

### Objective
Integrate different sensor types (camera, LiDAR, IMU) into a robot model.

### Tasks
1. Add a camera sensor to your robot
2. Add a LiDAR sensor to your robot
3. Add an IMU sensor to your robot
4. Configure ROS 2 plugins for each sensor

### Requirements
- Each sensor must have proper reference link
- ROS 2 plugins configured with appropriate namespaces
- Realistic sensor parameters
- Proper noise models for real-world simulation

### Steps
1. Modify your URDF from Exercise 2.2
2. Add Gazebo extensions for each sensor type
3. Configure plugins with ROS 2 integration
4. Test sensor data publication in ROS 2

### Deliverables
- Modified URDF with sensor integration
- List of sensor topics being published
- Sample sensor data output
- Explanation of sensor parameters

### Evaluation Criteria
- All sensors are properly integrated
- Sensor data is published to ROS 2 topics
- Parameters match realistic sensor specifications

---

## Exercise 2.5: Physics Tuning and Validation

### Objective
Tune physics parameters for realistic robot behavior and validate against real-world expectations.

### Tasks
1. Adjust friction and damping parameters
2. Tune collision surface properties
3. Validate robot behavior with different physics engines
4. Compare simulation vs. expected real-world behavior

### Requirements
- At least 2 different physics parameter sets
- Testing with ODE and Bullet engines
- Quantitative comparison of results
- Documentation of parameter effects

### Steps
1. Create a test world with ramps or obstacles
2. Implement different physics configurations
3. Run simulation tests with each configuration
4. Document the differences in behavior
5. Validate against expected physical behavior

### Deliverables
- Multiple physics configuration files
- Comparison table of simulation results
- Analysis of parameter effects
- Recommendations for physics settings

### Evaluation Criteria
- Physics parameters are appropriately tuned
- Simulation behavior is realistic
- Differences between configurations are documented

---

## Exercise 2.6: Complete Simulation Integration

### Objective
Create a complete simulation environment integrating all learned concepts.

### Tasks
1. Create a humanoid robot model with multiple joints
2. Design a custom world environment
3. Integrate multiple sensor types
4. Configure ROS 2 communication
5. Test the complete simulation

### Requirements
- Robot with at least 6 degrees of freedom
- World with obstacles and interactive elements
- All sensor types integrated
- Proper mass and inertial properties
- Working ROS 2 communication

### Steps
1. Design the complete robot URDF
2. Create the world file
3. Integrate sensors and plugins
4. Test the complete simulation
5. Document the setup process

### Deliverables
- Complete robot URDF
- Custom world file
- Configuration documentation
- Test results and validation
- Video or screenshots of the simulation

### Evaluation Criteria
- All components are properly integrated
- Simulation runs without errors
- Robot behaves realistically
- All sensors function correctly
- ROS 2 communication works as expected

---

## Self-Assessment Checklist

After completing these exercises, you should be able to:
- [ ] Set up Gazebo simulation environment
- [ ] Create and validate URDF robot models
- [ ] Configure custom Gazebo worlds
- [ ] Integrate various sensor types
- [ ] Tune physics parameters for realistic behavior
- [ ] Troubleshoot common simulation issues
- [ ] Validate simulation results against expectations