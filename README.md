Bio-Inspired Ant Navigation: The "Titan-Ant" Model
Introduction:-
This project demonstrates a decentralized, computation-light navigation architecture for autonomous warehouse robots. Drawing inspiration from the Cataglyphis desert ant, the simulation proves that complex spatial navigation can be achieved without heavy SLAM (Simultaneous Localization and Mapping) or centralized servers.
The "Titan-Ant" model combines Path Integration (Vector-Based Homing) with reactive Tactile Obstacle Negotiation, allowing agents to navigate cluttered, dynamic environments using minimal memory. Additionally, it implements Swarm Chirality rules to enable scalable multi-agent collision avoidance.
Features:-
Vector Homing (Path Integration): Agents calculate direct Euclidean vectors to targets, eliminating the need for global map storage.
Smart Bug Algorithm: A "Gap-Following" logic that detects obstacles via multi-ray whiskers and biases movement toward open spaces (Right/Left decision making).
Swarm Chirality: A decentralized "Right-Hand Traffic Rule" that prevents head-on collisions between autonomous agents.
Physics-Based Simulation: Built on PyBullet to model realistic collisions, sensor ray-casting, and kinematics.

Installation & Setup:-

Prerequisites:

Python 3.6+
pip

Step 1: Clone the Repository
git clone https://github.com/Harishkeran/TITAN-ANT

Step 2: Install Dependencies
This simulation requires pybullet for physics and numpy for vector calculations.
pip install pybullet numpy

How to Run:
Execute the main Python script to launch the simulation GUI:
python simulation.py

Controls: The simulation runs automatically. You can use your mouse to rotate/zoom the camera.
Visuals:
Red Line: Path history of the agent.
Blue Line: The "Home Vector" (Path Integration).
Yellow/Red Rays: Real-time sensor feedback (Yellow = Clear, Red = Obstacle).
Algorithm Description:
The robot's behavior is governed by a Hierarchical State Machine with three priority levels:
1. Primary State: Vector Homing
The robot continuously updates a "Home Vector" pointing to its assigned beacon (Green or Blue). It uses a Proportional (P) Controller to align its heading with this vector, ensuring robust navigation even after disturbances.
2. Secondary State: "Follow-Gap" Obstacle Negotiation
When the multi-ray whiskers detect an obstacle, the robot switches to a reactive mode. Instead of blindly following a wall, it compares sensor data from the Left and Right sides. It then biases its movement toward the "more open" direction, effectively flowing around clutter rather than getting stuck in corners.
3. Tertiary State: Collective Chirality
To handle multi-agent scenarios, the system enforces a local traffic rule. If two agents detect a collision course, both trigger a synchronized Negative Angular Velocity (Turn Right). This induces a smooth "swerving" maneuver, allowing high-speed movement in congested aisles without communication overhead.
