# Used https://stackoverflow.com/questions/21030391/how-to-normalize-a-numpy-array-to-a-unit-vector as a ref.
# and https://github.com/bulletphysics/bullet3/tree/master/examples 


import pybullet as p
import pybullet_data
import numpy as n
import pybullet as p
import pybullet_data
import time
import math
from sensors import Sensor
import pyb_utils
import cv2 as cv
import matplotlib.pyplot as plt

# success.wav by grunz -- https://freesound.org/s/109662/ -- License: Attribution 3.0
# Negative.wav by iwanPlays -- https://freesound.org/s/626085/ -- License: Creative Commons 0
    

class Simulation:
    def __init__(self):
        self.timer = 0
        
        self.offsetX = 0
        self.ballId = 0
        self.red_ball = 0
        self.robot = 0
        self.distFromGoal = 12
        self.startBallHeight = 0.2
        # targwidth is for the x of where the ball is placed.
        self.targWidth = 10
        self.physicsClient = 0
        self.leftCamera = 0
        self.rightCamera = 0
        self.video = 0
        self.CVindex = 0
        
        
        self.goal_width = 7.32  
        self.goal_height = 2.44 
        self.goal_depth = 3.5   
        self.post_radius = 0.1
        
        self.centerPointOne = 0
        self.centerPointTwo = 0
        self.centerPointThree = 0
        
        self.expected_calced = False
        self.blue_ball = 0
        self.expected_location = None


        
    def getRandX(self):
        targWidth = 6.5
        return (n.random.rand() * targWidth) - (targWidth / 2)

    def normalize(self, a):
        tmp = []
        mag = math.sqrt(a[0]**2 + a[1]**2 + a[2]**2)
        for i in range(3):
            tmp.append(a[i] / mag)
        return tmp

    def randomVecAtGoal(self):
        
        # DO math here to calc the speed and goal.
        
        # create vect at goal point rand
        # normalize and subtract starting loc
        
        # generate vec
        # 7.32 is the cross bar width 0.1 for cross bar radius and ball radius is estimated
        # the robot should be able to block all of the shots on goal.
        # Therefore, the Z axis isn't set to the height of the goal.
        #p.resetBasePositionAndOrientation(ballId,posObj=[offsetX, self.distFromGoal, self.startBallHeight], ornObj=[0,0,0,0] )
        # limiting goal to one spot.
        self.offsetX = 0
        p.removeBody(self.ballId)
        self.ballId = p.loadURDF("soccerball.urdf", [self.offsetX, self.distFromGoal, self.startBallHeight], globalScaling=0.22)
        p.changeDynamics(self.ballId, linkIndex=-1, linearDamping=0, angularDamping=0, mass=0.000000001)
        point_on_goal = [(n.random.rand() * 6.9 ) - (6.9 / 2),0,(n.random.rand() * 2) + 0.2]
        
        
        
        point_ball_start = [self.offsetX, self.distFromGoal, self.startBallHeight]
        if self.red_ball != 0:
            p.removeBody(self.red_ball)
        self.red_ball = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=p.createVisualShape(
                shapeType=p.GEOM_SPHERE,
                radius=0.11,
                rgbaColor=[1, 0, 0, 1],
            ),
            
            basePosition=point_on_goal,
            baseOrientation=p.getQuaternionFromEuler([0, n.pi / 2, 0])

        )

        ballVelo_to_be_norm = [
            point_on_goal[0] - point_ball_start[0],
            point_on_goal[1] - point_ball_start[1],
            point_on_goal[2] - point_ball_start[2]
        ]
        ballVelo = self.normalize(ballVelo_to_be_norm)
        # print(ballVelo)
        # potentially scale velo
        for index in range(3):
            ballVelo[index] = float(ballVelo[index]) * 7
            
        # try to get ball to go through point.
        t = 1.0
        ballVelo = [(point_on_goal[0] - point_ball_start[0])/t, (point_on_goal[1] - point_ball_start[1])/t, (point_on_goal[2] - point_ball_start[2] + (0.5*9.8 * (t**2)))/t]
        p.resetBaseVelocity(self.ballId, linearVelocity=ballVelo, angularVelocity=[-60, 0, 0])
    
    # inspiration from https://youtu.be/VDBSH69uH7A?si=8W3odvU4Xx_0HbTx&t=257
    # then solve system of equations
    def getEstBallLoc(self, points: n.ndarray):
        # print(points)
        # print(type(points))
        if len(points) > 4:
            tmp_a = []
            tmp_b = []
            base_point = points[0]
            bp_squared = n.dot(base_point,base_point)
            for ele in points[1:]:
                tmp_a.append(2 * (ele - base_point))
                tmp_b.append(n.dot(ele,ele) - bp_squared)
            np_A = n.array(tmp_a)
            np_b = n.array(tmp_b)
            
            # center is only relevant here.
            center, _, _, _ = n.linalg.lstsq(np_A, np_b, rcond=None)
            # print("actual")
            # print(p.getBasePositionAndOrientation(self.ballId))

            # print("center:")
            if type(self.centerPointOne) == int:
                self.centerPointOne = center
            elif type(self.centerPointTwo) == int:
                self.centerPointTwo = center
            elif type(self.centerPointThree) == int:
                self.centerPointThree = center
                self.calcExpectedVec()
                
            
            # print(center)
            
            
            
    def captureFrameCV(self):
        rgba, depth, seg = self.leftCamera.get_frame()
        # generate point cloud
        points = self.leftCamera.get_point_cloud(depth=depth)

        # just get points on the ball
        mask = (seg == self.ballId)
        points = points[mask, :]


        self.getEstBallLoc(points)
        
        
        
        
        # UNCOMMENT FOR VISUAL REPRESENTATION OF LIDAR
        
        # fig = plt.figure()
        # ax = fig.add_subplot(projection="3d")
        # ax.scatter(points[:, 0], points[:, 1], zs=points[:, 2])
        # ax.set_xlabel("x")
        # ax.set_ylabel("y")
        # ax.set_zlabel("z")
        # ax.set_aspect("equal")
        # plt.show()

        
    def initSensors(self):
        # refed from https://github.com/adamheins/pyb_utils/blob/main/examples/camera_example.py
        
        self.leftCamera = pyb_utils.Camera.from_camera_position( 
            camera_position=(0, 0, self.goal_height),
            target_position=(self.offsetX, self.distFromGoal, self.startBallHeight),
            near=0.1,
            far=100,
            fov=10.,
            width=200,
            height=200,
        )
        

        # save the frame
        # self.video = pyb_utils.camera.FrameRecorder(camera=self.leftCamera, fps=60)

    def calcExpectedVec(self):
        # make basis 0,0,0 as the cam isn't there.
        path = 0
        t1 = 1. / 240.
        t2 = 2. / 240.
    
        g = n.array([0,0, -9.8])
        v1 = (self.centerPointTwo - self.centerPointOne - 0.5 * g * t1**2) / t1
        v2 = (self.centerPointThree - self.centerPointOne - 0.5 * g * t2**2) / t2
        calc_init_velo = 0.5 * (v1 + v2)
        t = (0 - self.centerPointOne[1]) / calc_init_velo[1]

        
        
        expected_location = n.array([
            self.centerPointOne[0] + calc_init_velo[0] * t,
            0,
            (self.centerPointOne[2] + (calc_init_velo[2] * t)) + (0.5 * -9.8 * t**2)
        ])
        self.expected_location = expected_location  # Save for base control

        if self.blue_ball != 0:
            p.removeBody(self.blue_ball)
        self.blue_ball = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=p.createVisualShape(
                shapeType=p.GEOM_SPHERE,
                radius=0.11,
                rgbaColor=[0, 0, 1, 1],
            ),
            
            basePosition=expected_location,
            baseOrientation=p.getQuaternionFromEuler([0, n.pi / 2, 0])

        )  
        self.expected_calced = True

    def reach_with_arms(self, target_pos):
        if not hasattr(self, 'robot'):
            return

        joint_map = {p.getJointInfo(self.robot, i)[1].decode(): i for i in range(p.getNumJoints(self.robot))}
        link_map = {p.getJointInfo(self.robot, i)[12].decode(): i for i in range(p.getNumJoints(self.robot))}

        left_shoulder = joint_map.get("left_shoulder")
        left_elbow = joint_map.get("left_elbow")
        right_shoulder = joint_map.get("right_shoulder")
        right_elbow = joint_map.get("right_elbow")
        left_hand_link = link_map.get("left_hand")
        right_hand_link = link_map.get("right_hand")

        active_joints = [
            i for i in range(p.getNumJoints(self.robot))
            if p.getJointInfo(self.robot, i)[2] != p.JOINT_FIXED
        ]

        if None in [left_shoulder, left_elbow, right_shoulder, right_elbow, left_hand_link, right_hand_link]:
            print("Missing joints or hand links.")
            return

        base_pos, _ = p.getBasePositionAndOrientation(self.robot)
        rel_x = target_pos[0] - base_pos[0]

        def apply_ik(ik_solution, joint_names):
            joint_index_map = {j: i for i, j in enumerate(active_joints)}
            for name in joint_names:
                joint_id = joint_map[name]
                if joint_id in joint_index_map:
                    sol_idx = joint_index_map[joint_id]
                    p.setJointMotorControl2(
                        self.robot, joint_id, p.POSITION_CONTROL, targetPosition=ik_solution[sol_idx], force=100
                    )

        if rel_x < -0.2:
            ik = p.calculateInverseKinematics(self.robot, right_hand_link, target_pos)
            apply_ik(ik, ["right_shoulder", "right_elbow"])
        elif rel_x > 0.2:
            ik = p.calculateInverseKinematics(self.robot, left_hand_link, target_pos)
            apply_ik(ik, ["left_shoulder", "left_elbow"])
        else:
            ik_l = p.calculateInverseKinematics(self.robot, left_hand_link, target_pos)
            ik_r = p.calculateInverseKinematics(self.robot, right_hand_link, target_pos)
            apply_ik(ik_l, ["left_shoulder", "left_elbow"])
            apply_ik(ik_r, ["right_shoulder", "right_elbow"])



        
            
    def setup_soccer_goal_environment(self):
        """
        Sets up a PyBullet environment with a plane and a basic soccer goal.
        """
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        p.resetDebugVisualizerCamera(
            cameraDistance=10,          # Zoom
            cameraYaw=180,              # Left-right rotation (degrees)
            cameraPitch=-5,           # Up-down angle (degrees)
            cameraTargetPosition=[0, 0, 0]  # Where the camera looks
        )


        # Create the setting
        plane_id = p.loadURDF("plane.urdf")
        p.changeVisualShape(plane_id, -1, rgbaColor=[0, 0, 0, 0])  # Fully transparent


        # Define the goal dimensions
         
        field_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[70, 105, 0.01],
            rgbaColor=[144/255, 224/255, 72/255, 1], specularColor=[0, 0, 0] )

        field_id = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=field_visual,
            basePosition=[-35, -52, -0.01]
        )

        # Create goal
        goalpost1Id = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=p.createVisualShape(
                shapeType=p.GEOM_CYLINDER,
                radius=self.post_radius,
                length=self.goal_height,
                rgbaColor=[1, 1, 1, 1]
            ),
            baseCollisionShapeIndex=p.createCollisionShape(
                shapeType=p.GEOM_CYLINDER,
                radius=self.post_radius,
                height=self.goal_height
            ),
            basePosition=[self.goal_width / 2, 0, self.goal_height / 2],
        )

        goalpost2Id = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=p.createVisualShape(
                shapeType=p.GEOM_CYLINDER,
                radius=self.post_radius,
                length=self.goal_height,
                rgbaColor=[1, 1, 1, 1]
            ),
            baseCollisionShapeIndex=p.createCollisionShape(
                shapeType=p.GEOM_CYLINDER,
                radius=self.post_radius,
                height=self.goal_height
            ),
            basePosition=[-self.goal_width / 2, 0, self.goal_height / 2],
        )

        
        crossbarId = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=p.createVisualShape(
                shapeType=p.GEOM_CYLINDER,
                radius=self.post_radius,
                length=self.goal_width,
                rgbaColor=[1, 1, 1, 1],
            ),
            baseCollisionShapeIndex=p.createCollisionShape(
                shapeType=p.GEOM_CYLINDER,
                radius=self.post_radius,
                height=self.goal_width,
            ),
            basePosition=[0, 0, self.goal_height],
            baseOrientation=p.getQuaternionFromEuler([0, n.pi / 2, 0])

        )
        backstop_width = self.goal_width
        backstop_height = self.goal_height
        backstop_depth = 0.1
        backstopId = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=p.createVisualShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[backstop_width / 2, backstop_depth / 2, backstop_height / 2],
                rgbaColor=[0.8, 0.8, 0.8, 0.5]
            ),
            baseCollisionShapeIndex=p.createCollisionShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[backstop_width / 2, backstop_depth / 2, backstop_height / 2],
            ),
            basePosition=[0, -self.goal_depth / 2, self.goal_height / 2],
            baseOrientation=p.getQuaternionFromEuler([0, n.pi, 0])
        )
        
        leftBackstopId = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=p.createVisualShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[self.goal_width/8, backstop_depth / 2, backstop_height / 2],
                rgbaColor=[0.8, 0.8, 0.8, 0.5]
            ),
            baseCollisionShapeIndex=p.createCollisionShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[self.goal_width/8, backstop_depth / 2, backstop_height / 2],
            ),
            basePosition=[(self.goal_depth + 0.2), -self.goal_depth / 4, self.goal_height / 2],
            baseOrientation=p.getQuaternionFromEuler([0, 0, n.pi/2])
        )
        rightBackstopId = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=p.createVisualShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[self.goal_width/8, backstop_depth / 2, backstop_height / 2],
                rgbaColor=[0.8, 0.8, 0.8, 0.5]
            ),
            baseCollisionShapeIndex=p.createCollisionShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[self.goal_width/8, backstop_depth / 2, backstop_height / 2],
            ),
            basePosition=[-(self.goal_depth + 0.2), -self.goal_depth / 4, self.goal_height / 2],
            baseOrientation=p.getQuaternionFromEuler([0, 0, n.pi/2])
        )
        
        topbarL = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=p.createVisualShape(
                shapeType=p.GEOM_CYLINDER,
                radius=self.post_radius,
                length=self.goal_width/4,
                rgbaColor=[1, 1, 1, 1],
            ),
            baseCollisionShapeIndex=p.createCollisionShape(
                shapeType=p.GEOM_CYLINDER,
                radius=self.post_radius,
                height=self.goal_width/4,
            ),
            basePosition=[self.goal_depth, -self.goal_depth / 4, self.goal_height],
            baseOrientation=p.getQuaternionFromEuler([n.pi/2, 0, 0])

        )
        topbarR = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=p.createVisualShape(
                shapeType=p.GEOM_CYLINDER,
                radius=self.post_radius,
                length=self.goal_width/4,
                rgbaColor=[1, 1, 1, 1],
            ),
            baseCollisionShapeIndex=p.createCollisionShape(
                shapeType=p.GEOM_CYLINDER,
                radius=self.post_radius,
                height=self.goal_width/4,
            ),
            basePosition=[-self.goal_depth, -self.goal_depth / 4, self.goal_height],
            baseOrientation=p.getQuaternionFromEuler([n.pi/2, 0, 0])

        )
        
        backCrossbarId = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=p.createVisualShape(
                shapeType=p.GEOM_CYLINDER,
                radius=self.post_radius,
                length=self.goal_width,
                rgbaColor=[1, 1, 1, 1],
            ),
            baseCollisionShapeIndex=p.createCollisionShape(
                shapeType=p.GEOM_CYLINDER,
                radius=self.post_radius,
                height=self.goal_width,
            ),
            basePosition=[0, -self.goal_depth / 2, self.goal_height],
            baseOrientation=p.getQuaternionFromEuler([0, n.pi / 2, 0])

        )
        
        # Load a soccer ball

        self.offsetX = 0
        self.ballId = p.loadURDF("soccerball.urdf", [self.offsetX, self.distFromGoal, self.startBallHeight], globalScaling=0.22)
        
        self.randomVecAtGoal()
        self.robot = p.loadURDF("robots/simple.urdf", [0,0,0.6])
        p.changeDynamics(self.robot, linkIndex=-1, mass=2000, restitution=0.8)
        self.initSensors()
        return self.physicsClient  # return the ID, so the user can use it

    def simulate_environment(self, physicsClient):
        """
        Runs the simulation loop.

        Args:
            physicsClient: The physics client ID returned from setup_soccer_goal_environment().
        """
        
        keypressed = False
        self.timer = time.time()
        try:
            while True:
                
                # TODO: CHECK IF BALL IS IN GOAL. IF SO RESET SIM
                pos, orn = p.getBasePositionAndOrientation(self.ballId)
                aabb_min, aabb_max = p.getAABB(self.ballId)
                # print(aabb_max, aabb_min)
                if aabb_max[1] < 0 and (pos[0] < (7.22 / 2) and pos[0] > (-7.22 / 2)) and (pos[2] < 2.44):
                    print("GOAL!")
                    for _ in range(120):
                        time.sleep(1. / 240.)
                        p.stepSimulation()
                    
                    self.expected_calced = False
                    self.centerPointOne = 0
                    self.centerPointTwo = 0
                    self.centerPointThree = 0
                    self.timer = time.time()

                    self.offsetX = 0
                    
                    self.randomVecAtGoal()
                # TODO: KEEP TRACKER IN TERMINAL OF TRIALS ie: "succeed: 1, fail: 0"
                elif time.time() - self.timer > 3 and keypressed:
                    print("MISS")
                    self.timer = time.time()
                    
                    self.centerPointThree = 0
                    self.centerPointOne = 0
                    self.centerPointTwo = 0
                    self.expected_calced = False                   
                    # self.offsetX = self.getRandX()
                    
                    self.randomVecAtGoal()
                
                if keypressed:
                    p.stepSimulation()
                    if not self.expected_calced:
                        self.captureFrameCV()
                    else:
                        # Move base to align with expected ball position on the x-axis
                        if hasattr(self, 'expected_location'):
                            target_x = self.expected_location[0]
                            current_pos, current_orn = p.getBasePositionAndOrientation(self.robot)
                            current_x = current_pos[0]

                            # Limit how close the base can get to the target x (stop at ~1m away)
                            distance = abs(target_x - current_x)
                            approach_limit = 0.8  # stop when this close

                            if distance > approach_limit:
                                direction = (target_x - current_x) / distance
                                step_size = 0.05
                                new_x = current_x + step_size * direction
                            else:
                                new_x = current_x  # stay here and let the arms do the work

                            new_pos = [new_x, current_pos[1], current_pos[2]]
                            p.resetBasePositionAndOrientation(self.robot, new_pos, current_orn)
                            self.reach_with_arms(self.expected_location)



                # insert checks here.
                else:
                    if input() == "S":
                        keypressed = True
                        self.timer = time.time()
                time.sleep(1. / 240.)
        except KeyboardInterrupt:
            print("Exiting simulation...")
        finally:
            p.disconnect()

if __name__ == "__main__":
    # Set up the environment
    sim = Simulation()
    client_id = sim.setup_soccer_goal_environment()
    # Run the simulation
    sim.simulate_environment(client_id)
    
    
    
