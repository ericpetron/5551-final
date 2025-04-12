import pybullet as p
import pybullet_data
import numpy as n
import pybullet as p
import pybullet_data
import time

def setup_soccer_goal_environment():
    """
    Sets up a PyBullet environment with a plane and a basic soccer goal.
    """
    physicsClient = p.connect(p.GUI)  # Use p.GUI for graphical interface, or p.DIRECT for headless
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)

    # Create a plane
    planeId = p.loadURDF("plane.urdf")

    # Define the goal dimensions
    goal_width = 7.32  
    goal_height = 2.44 
    goal_depth = 3.5   
    post_radius = 0.1 

    # Create the goal posts (cylinders)
    goalpost1Id = p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=p.createVisualShape(
            shapeType=p.GEOM_CYLINDER,
            radius=post_radius,
            length=goal_height,
            rgbaColor=[1, 1, 1, 1]  # White color
        ),
        baseCollisionShapeIndex=p.createCollisionShape(
            shapeType=p.GEOM_CYLINDER,
            radius=post_radius,
            height=goal_height
        ),
        basePosition=[goal_width / 2, 0, goal_height / 2],  # Position of one post
    )

    goalpost2Id = p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=p.createVisualShape(
            shapeType=p.GEOM_CYLINDER,
            radius=post_radius,
            length=goal_height,
            rgbaColor=[1, 1, 1, 1]  # White color
        ),
        baseCollisionShapeIndex=p.createCollisionShape(
            shapeType=p.GEOM_CYLINDER,
            radius=post_radius,
            height=goal_height
        ),
        basePosition=[-goal_width / 2, 0, goal_height / 2],  # Position of the other post
    )

    # Create the crossbar (cylinder)
    crossbarId = p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=p.createVisualShape(
            shapeType=p.GEOM_CYLINDER,
            radius=post_radius,
            length=goal_width,
            rgbaColor=[1, 1, 1, 1],  # White
        ),
        baseCollisionShapeIndex=p.createCollisionShape(
            shapeType=p.GEOM_CYLINDER,
            radius=post_radius,
            height=goal_width,
        ),
        basePosition=[0, 0, goal_height],  # Position of the crossbar
        baseOrientation=p.getQuaternionFromEuler([0, n.pi / 2, 0])  # Rotate it.

    )
    # Optional: Add a simple backstop.  This makes it easier to see if the ball goes in.
    backstop_width = goal_width
    backstop_height = goal_height
    backstop_depth = 0.1
    backstopId = p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=p.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[backstop_width / 2, backstop_depth / 2, backstop_height / 2],
            rgbaColor=[0.8, 0.8, 0.8, 0.5]  # Grey and semi-transparent
        ),
        baseCollisionShapeIndex=p.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[backstop_width / 2, backstop_depth / 2, backstop_height / 2],
        ),
        basePosition=[0, -goal_depth / 2, goal_height / 2],  # Position the backstop behind the goal
        baseOrientation=p.getQuaternionFromEuler([0, n.pi, 0])
    )

    # Load a soccer ball
    ballId = p.loadURDF("soccerball.urdf", [0, 5, 1], globalScaling=0.5)
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
    
    
    
def randomVecAtGoat(ballID):
    
    # DO math here to calc the speed and goal.
    
    
    
    p.resetBaseVelocity(ballID, linearVelocity=[x, y, z], angularVelocity=[wx, wy, wz])
