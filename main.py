# Used https://stackoverflow.com/questions/21030391/how-to-normalize-a-numpy-array-to-a-unit-vector as a ref.
# and https://github.com/bulletphysics/bullet3/tree/master/examples 


import pybullet as p
import pybullet_data
import numpy as n
import pybullet as p
import pybullet_data
import time
import math
def normalize(a):
    tmp = []
    mag = math.sqrt(a[0]**2 + a[1]**2 + a[2]**2)
    for i in range(3):
        tmp.append(a[i] / mag)
    return tmp

def randomVecAtGoal(ballID):
    
    # DO math here to calc the speed and goal.
    
    # create vect at goal point rand
    # normalize and subtract starting loc
    
    # generate vec
    # 7.32 is the cross bar width 0.1 for cross bar radius and ball radius is estimated
    # the robot should be able to block all of the shots on goal.
    # Therefore, the Z axis isn't set to the height of the goal.
    point_on_goal = [(n.random.rand() * 6.9 ) - (6.9 / 2),0,(n.random.rand() * 2)]
    point_ball_start = [0, 5, 1]
    backCrossbarId = p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=p.createVisualShape(
            shapeType=p.GEOM_SPHERE,
            radius=0.1,
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
    ballVelo = normalize(ballVelo_to_be_norm)
    print(ballVelo)
    # potentially scale velo
    for index in range(3):
        ballVelo[index] = float(ballVelo[index]) * 7
    p.resetBaseVelocity(ballID, linearVelocity=ballVelo, angularVelocity=[0, 0, 0])


def setup_soccer_goal_environment():
    """
    Sets up a PyBullet environment with a plane and a basic soccer goal.
    """
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)

    # Create the setting
    planeId = p.loadURDF("plane.urdf")

    # Define the goal dimensions
    goal_width = 7.32  
    goal_height = 2.44 
    goal_depth = 3.5   
    post_radius = 0.1 

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
    ballId = p.loadURDF("soccerball.urdf", [0, 5, 1], globalScaling=0.5)
    randomVecAtGoal(ballId)
    robot = p.loadURDF("robots/simple.urdf", [0,0,1])

    return physicsClient  # return the ID, so the user can use it

def simulate_environment(physicsClient):
    """
    Runs the simulation loop.

    Args:
        physicsClient: The physics client ID returned from setup_soccer_goal_environment().
    """
    try:
        while True:
            p.stepSimulation()
            # insert checks here.
            # TODO: CHECK IF BALL IS IN GOAL. IF SO RESET SIM
            
            # TODO: KEEP TRACKER IN TERMINAL OF TRIALS ie: "succeed: 1, fail: 0"
            
            # TODO: 
            
            time.sleep(1. / 240.)
    except KeyboardInterrupt:
        print("Exiting simulation...")
    finally:
        p.disconnect()

if __name__ == "__main__":
    # Set up the environment
    client_id = setup_soccer_goal_environment()
    # Run the simulation
    simulate_environment(client_id)
    
    
    
