# Used https://stackoverflow.com/questions/21030391/how-to-normalize-a-numpy-array-to-a-unit-vector as a ref.
# and https://github.com/bulletphysics/bullet3/tree/master/examples 


import pybullet as p
import pybullet_data
import numpy as n
import pybullet as p
import pybullet_data
import time
import math

# success.wav by grunz -- https://freesound.org/s/109662/ -- License: Attribution 3.0
# Negative.wav by iwanPlays -- https://freesound.org/s/626085/ -- License: Creative Commons 0
    

class Simulation:
    def __init__(self):
        self.offsetX = 0
        self.ballId = 0
        self.red_ball = 0
        self.robot = 0
        self.distFromGoal = 10
        self.startBallHeight = 0.2
        # targwidth is for the x of where the ball is placed.
        self.targWidth = 10
        

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
        print(ballVelo)
        # potentially scale velo
        for index in range(3):
            ballVelo[index] = float(ballVelo[index]) * 7
            
        # try to get ball to go through point.
        t = 1.0
        ballVelo = [(point_on_goal[0] - point_ball_start[0])/t, (point_on_goal[1] - point_ball_start[1])/t, (point_on_goal[2] - point_ball_start[2] + (0.5*9.8 * (t**2)))/t]
        p.resetBaseVelocity(self.ballId, linearVelocity=ballVelo, angularVelocity=[-60, 0, 0])


    def setup_soccer_goal_environment(self):
        """
        Sets up a PyBullet environment with a plane and a basic soccer goal.
        """
        physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        

        # Create the setting
        plane_id = p.loadURDF("plane.urdf")
        p.changeVisualShape(plane_id, -1, rgbaColor=[0, 0, 0, 0])  # Fully transparent


        # Define the goal dimensions
        goal_width = 7.32  
        goal_height = 2.44 
        goal_depth = 3.5   
        post_radius = 0.1 
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
                radius=post_radius,
                length=goal_height,
                rgbaColor=[1, 1, 1, 1]
            ),
            baseCollisionShapeIndex=p.createCollisionShape(
                shapeType=p.GEOM_CYLINDER,
                radius=post_radius,
                height=goal_height
            ),
            basePosition=[goal_width / 2, 0, goal_height / 2],
        )

        goalpost2Id = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=p.createVisualShape(
                shapeType=p.GEOM_CYLINDER,
                radius=post_radius,
                length=goal_height,
                rgbaColor=[1, 1, 1, 1]
            ),
            baseCollisionShapeIndex=p.createCollisionShape(
                shapeType=p.GEOM_CYLINDER,
                radius=post_radius,
                height=goal_height
            ),
            basePosition=[-goal_width / 2, 0, goal_height / 2],
        )

        
        crossbarId = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=p.createVisualShape(
                shapeType=p.GEOM_CYLINDER,
                radius=post_radius,
                length=goal_width,
                rgbaColor=[1, 1, 1, 1],
            ),
            baseCollisionShapeIndex=p.createCollisionShape(
                shapeType=p.GEOM_CYLINDER,
                radius=post_radius,
                height=goal_width,
            ),
            basePosition=[0, 0, goal_height],
            baseOrientation=p.getQuaternionFromEuler([0, n.pi / 2, 0])

        )
        backstop_width = goal_width
        backstop_height = goal_height
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
            basePosition=[0, -goal_depth / 2, goal_height / 2],
            baseOrientation=p.getQuaternionFromEuler([0, n.pi, 0])
        )
        
        leftBackstopId = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=p.createVisualShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[goal_width/8, backstop_depth / 2, backstop_height / 2],
                rgbaColor=[0.8, 0.8, 0.8, 0.5]
            ),
            baseCollisionShapeIndex=p.createCollisionShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[goal_width/8, backstop_depth / 2, backstop_height / 2],
            ),
            basePosition=[(goal_depth + 0.2), -goal_depth / 4, goal_height / 2],
            baseOrientation=p.getQuaternionFromEuler([0, 0, n.pi/2])
        )
        rightBackstopId = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=p.createVisualShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[goal_width/8, backstop_depth / 2, backstop_height / 2],
                rgbaColor=[0.8, 0.8, 0.8, 0.5]
            ),
            baseCollisionShapeIndex=p.createCollisionShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[goal_width/8, backstop_depth / 2, backstop_height / 2],
            ),
            basePosition=[-(goal_depth + 0.2), -goal_depth / 4, goal_height / 2],
            baseOrientation=p.getQuaternionFromEuler([0, 0, n.pi/2])
        )
        
        topbarL = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=p.createVisualShape(
                shapeType=p.GEOM_CYLINDER,
                radius=post_radius,
                length=goal_width/4,
                rgbaColor=[1, 1, 1, 1],
            ),
            baseCollisionShapeIndex=p.createCollisionShape(
                shapeType=p.GEOM_CYLINDER,
                radius=post_radius,
                height=goal_width/4,
            ),
            basePosition=[goal_depth, -goal_depth / 4, goal_height],
            baseOrientation=p.getQuaternionFromEuler([n.pi/2, 0, 0])

        )
        topbarR = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=p.createVisualShape(
                shapeType=p.GEOM_CYLINDER,
                radius=post_radius,
                length=goal_width/4,
                rgbaColor=[1, 1, 1, 1],
            ),
            baseCollisionShapeIndex=p.createCollisionShape(
                shapeType=p.GEOM_CYLINDER,
                radius=post_radius,
                height=goal_width/4,
            ),
            basePosition=[-goal_depth, -goal_depth / 4, goal_height],
            baseOrientation=p.getQuaternionFromEuler([n.pi/2, 0, 0])

        )
        
        backCrossbarId = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=p.createVisualShape(
                shapeType=p.GEOM_CYLINDER,
                radius=post_radius,
                length=goal_width,
                rgbaColor=[1, 1, 1, 1],
            ),
            baseCollisionShapeIndex=p.createCollisionShape(
                shapeType=p.GEOM_CYLINDER,
                radius=post_radius,
                height=goal_width,
            ),
            basePosition=[0, -goal_depth / 2, goal_height],
            baseOrientation=p.getQuaternionFromEuler([0, n.pi / 2, 0])

        )
        
        # Load a soccer ball

        self.offsetX = self.getRandX()
        self.ballId = p.loadURDF("soccerball.urdf", [self.offsetX, self.distFromGoal, self.startBallHeight], globalScaling=0.22)
        
        self.randomVecAtGoal()
        self.robot = p.loadURDF("robots/simple.urdf", [0,0,1])
        p.changeDynamics(self.robot, linkIndex=-1, mass=2000, restitution=0.8)

        return physicsClient  # return the ID, so the user can use it

    def simulate_environment(self, physicsClient):
        """
        Runs the simulation loop.

        Args:
            physicsClient: The physics client ID returned from setup_soccer_goal_environment().
        """
        
        keypressed = False
        timer = time.time()
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
                    
                    
                    timer = time.time()

                    self.offsetX = self.getRandX()
                    
                    self.randomVecAtGoal()
                # TODO: KEEP TRACKER IN TERMINAL OF TRIALS ie: "succeed: 1, fail: 0"
                elif time.time() - timer > 3 and keypressed:
                    print("MISS")
                    timer = time.time()

                    self.offsetX = self.getRandX()
                    
                    self.randomVecAtGoal()
                # TODO: 
                if keypressed:
                    p.stepSimulation()
                # insert checks here.
                else:
                    if input() == "S":
                        keypressed = True
                        timer = time.time()
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
    
    
    
