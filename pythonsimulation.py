"""
Bio-Inspired Swarm Navigation Simulation
Project: Ant-Like Celestial Navigation in Cluttered Warehouses
Behaviors: Path Integration, 'Follow-Gap' Bug Algorithm, Collective Chirality
"""

import pybullet as p
import pybullet_data
import time
import math
import numpy as np

# --- SIMULATION CONFIGURATION ---
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0) # Clean view

# Top-down camera for trajectory analysis
p.resetDebugVisualizerCamera(cameraDistance=14, cameraYaw=0, cameraPitch=-89.9, cameraTargetPosition=[0, 0, 0])
p.loadURDF("plane.urdf")

# --- ENVIRONMENT SETUP ---
def create_environment():
    """Builds warehouse with beacons (goals) and static obstacles."""
    
    # Destinations (Beacons)
    green_goal = p.createVisualShape(p.GEOM_CYLINDER, radius=0.6, length=0.1, rgbaColor=[0, 1, 0, 0.6])
    p.createMultiBody(baseVisualShapeIndex=green_goal, basePosition=[6, 6, 0.05])
    
    blue_goal = p.createVisualShape(p.GEOM_CYLINDER, radius=0.6, length=0.1, rgbaColor=[0, 0, 1, 0.6])
    p.createMultiBody(baseVisualShapeIndex=blue_goal, basePosition=[6, -6, 0.05])

    # Static Obstacles
    # 1. Central Wall (Forces Red Robot to negotiate)
    wall_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 2.5, 0.5])
    wall_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.2, 2.5, 0.5], rgbaColor=[0.2, 0.2, 0.2, 1])
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_col, baseVisualShapeIndex=wall_vis, basePosition=[2, 0, 0.5])

    # 2. Scattered Boxes
    box_positions = [[-3, 2, 0.5], [-3, -2, 0.5]]
    for pos in box_positions:
        b_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.6, 0.6, 0.5])
        b_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.6, 0.6, 0.5], rgbaColor=[0.5, 0.3, 0.1, 1])
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=b_col, baseVisualShapeIndex=b_vis, basePosition=pos)

    # 3. Obstacle in Blue Robot's Path
    rand_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.5])
    rand_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.5], rgbaColor=[0.8, 0.5, 0.2, 1])
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=rand_col, baseVisualShapeIndex=rand_vis, basePosition=[3.5, -5, 0.5])

create_environment()

# --- AUTONOMOUS AGENT CLASS ---
class BioRobot:
    def __init__(self, start_pos, target_pos, color, name):
        self.target_pos = target_pos
        self.color = color
        self.name = name
        
        # Physics Body
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.25, 0.2, 0.1])
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.25, 0.2, 0.1], rgbaColor=color)
        self.id = p.createMultiBody(baseMass=10, baseCollisionShapeIndex=col, 
                                    baseVisualShapeIndex=vis, basePosition=start_pos)
        
        # State Variables
        self.path_history = []
        self.prev_pos = start_pos
        self.state = "HOMING"
        self.bug_timer = 0
        self.text_id = p.addUserDebugText(name, [0,0,0.5], [0,0,0])

    def get_kinematics(self):
        """Returns (x,y,z) and Yaw heading."""
        pos, orn = p.getBasePositionAndOrientation(self.id)
        yaw = p.getEulerFromQuaternion(orn)[2]
        return list(pos), yaw

    def sense_whiskers(self):
        """Simulates antennae: Returns hit distances for [Left, Center, Right]."""
        pos, yaw = self.get_kinematics()
        ray_len = 3.0
        hits = []
        angles = [0.5, 0, -0.5] # Left (+30deg), Center (0), Right (-30deg)
        
        for angle_offset in angles:
            check_yaw = yaw + angle_offset
            start_pos = pos
            end_x = pos[0] + ray_len * math.cos(check_yaw)
            end_y = pos[1] + ray_len * math.sin(check_yaw)
            end_pos = [end_x, end_y, pos[2]]
            
            res = p.rayTest(start_pos, end_pos)
            hit_val = res[0][2] # 1.0 = Clear, 0.0 = Collision
            hits.append(hit_val)
            
            # Visual Debug: Yellow = Clear, Red = Obstacle
            color = [1, 0, 0] if hit_val < 0.99 else [1, 1, 0]
            p.addUserDebugLine(start_pos, end_pos, color, lifeTime=0.1)
            
        return hits 

    def update(self, all_robots):
        """Main Logic Loop: Sensing -> Decision -> Actuation."""
        pos, yaw = self.get_kinematics()
        
        # --- VISUALIZATION & MEMORY ---
        if math.dist(pos, self.prev_pos) > 0.15:
            self.path_history.append(pos)
            if len(self.path_history) > 1:
                p.addUserDebugLine(self.path_history[-2], self.path_history[-1], self.color[:3], lifeTime=0)
            self.prev_pos = pos
            
        p.removeUserDebugItem(self.text_id)
        self.text_id = p.addUserDebugText(f"{self.state}", [pos[0], pos[1], 0.8], [0,0,0], textSize=1.0)
        
        # Path Integration Vector (Blue line to Home)
        p.addUserDebugLine(pos, [self.target_pos[0], self.target_pos[1], 0.5], self.color[:3], lifeTime=0.1)

        # --- SENSING ---
        dx = self.target_pos[0] - pos[0]
        dy = self.target_pos[1] - pos[1]
        dist_to_target = math.sqrt(dx**2 + dy**2)
        angle_to_target = math.atan2(dy, dx)
        
        # Whiskers: [Left, Center, Right]
        w_left, w_center, w_right = self.sense_whiskers()

        # Swarm Sensing (Proximity Check)
        swarm_risk = False
        for other in all_robots:
            if other.id == self.id: continue
            other_pos, _ = other.get_kinematics()
            if math.dist(pos[:2], other_pos[:2]) < 2.5: 
                # Check relative angle
                angle_to_other = math.atan2(other_pos[1]-pos[1], other_pos[0]-pos[0])
                angle_diff = (angle_to_other - yaw + math.pi) % (2*math.pi) - math.pi
                if abs(angle_diff) < 1.0: 
                    swarm_risk = True

        # --- HIERARCHICAL STATE MACHINE ---
        linear_vel = 0
        angular_vel = 0

        # 1. SWARM AVOIDANCE (Highest Priority)
        # Chirality Rule: Both agents turn Right to avoid head-on collision
        if swarm_risk:
            self.state = "SWARM_AVOID"
            linear_vel = 2.0
            angular_vel = -2.5 

        # 2. OBSTACLE NEGOTIATION (Follow-Gap / Bug Algorithm)
        elif self.state == "BUG_MODE" or ((w_center < 0.5 or w_left < 0.5 or w_right < 0.5) and self.state != "ARRIVED"):
            self.state = "BUG_MODE"
            
            # A. Emergency Reverse
            if w_center < 0.15 or w_left < 0.15 or w_right < 0.15:
                linear_vel = -1.0
                angular_vel = 0
            
            # B. Gap Finding Logic
            # If Left is clearer than Right, go Left. Otherwise, go Right.
            elif w_left > w_right: 
                if w_center < 0.5: # Blocked ahead
                    linear_vel = 0.5
                    angular_vel = 3.0 # Pivot Left
                else: # Clear ahead, but obstacle on Right
                    linear_vel = 3.0
                    angular_vel = 1.0 # Arc Left (Follow wall)
            else: # Right is clearer
                if w_center < 0.5:
                    linear_vel = 0.5
                    angular_vel = -3.0 # Pivot Right
                else:
                    linear_vel = 3.0
                    angular_vel = -1.0 # Arc Right (Follow wall)

            # C. Leave Condition
            # Resume homing only if path is clear and we face the target
            angle_diff_target = (angle_to_target - yaw + math.pi) % (2*math.pi) - math.pi
            if abs(angle_diff_target) < 0.8 and w_center > 0.9:
                self.bug_timer += 1
                if self.bug_timer > 15: 
                    self.state = "HOMING"
                    self.bug_timer = 0
            else:
                self.bug_timer = 0

        # 3. VECTOR HOMING (Path Integration)
        else:
            if dist_to_target < 0.8:
                self.state = "ARRIVED"
                linear_vel = 0
                angular_vel = 2.0 # Idle spin
            else:
                self.state = "HOMING"
                angle_diff = (angle_to_target - yaw + math.pi) % (2*math.pi) - math.pi
                angular_vel = angle_diff * 4.0 # P-Controller
                linear_vel = 4.0

        # --- ACTUATION ---
        vel_vector = [linear_vel * math.cos(yaw), linear_vel * math.sin(yaw), 0]
        p.resetBaseVelocity(self.id, linearVelocity=vel_vector, angularVelocity=[0, 0, angular_vel])

# --- MAIN EXECUTION ---
# Red: Forager (Bottom-Left -> Top-Right)
bot_red = BioRobot([-5, -5, 0.15], [6, 6], [0.9, 0.1, 0.1, 1], "Forager")
# Blue: Worker (Top-Left -> Bottom-Right)
bot_blue = BioRobot([-5, 5, 0.15], [6, -6], [0.1, 0.1, 0.9, 1], "Worker")

robots = [bot_red, bot_blue]

print("Simulation Running...")
while True:
    p.stepSimulation()
    for bot in robots:
        bot.update(robots)
    time.sleep(1./60.)