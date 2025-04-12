import pybullet as p
import pybullet_data
import time
import numpy as n

def setup_and_control_robot(urdf_path="humanoid_robot.urdf", initial_joint_angles=None):
    """
    Sets up a PyBullet simulation, loads a robot from a URDF, and controls its joints in a loop.

    Args:
        urdf_path (str, optional): Path to the URDF file. Defaults to "humanoid_robot.urdf".
        initial_joint_angles (dict, optional): Dictionary of initial joint angles
            {joint_name: angle_in_radians}.  If None, no initial pose is set.
    """
    # physicsClient = p.connect(p.GUI)  # Use p.GUI for graphical interface, or p.DIRECT for headless
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)

    planeId = p.loadURDF("plane.urdf")
    robotId = p.loadURDF(urdf_path, [0, 0, 0.1])

    numJoints = p.getNumJoints(robotId)
    print(f"Number of joints: {numJoints}")

    jointNameToId = {}
    for i in range(numJoints):
        jointInfo = p.getJointInfo(robotId, i)
        jointName = jointInfo[1].decode('UTF-8')
        jointId = jointInfo[0]
        jointType = jointInfo[2]
        jointNameToId[jointName] = jointId
        print(f"Joint {i}: Name={jointName}, ID={jointId}, Type={jointType}")

    # Initialize the robot pose if initial joint angles are provided
    if initial_joint_angles:
        for joint_name, angle in initial_joint_angles.items():
            if joint_name in jointNameToId:
                joint_id = jointNameToId[joint_name]
                p.resetJointState(robotId, joint_id, angle)
            else:
                print(f"Warning: Joint '{joint_name}' not found in the robot model.")

    # Get the joint IDs for the joints we want to control.  This is now more flexible.
    shoulder_left_id = jointNameToId.get('left_shoulder')
    elbow_left_id = jointNameToId.get('left_elbow')
    shoulder_right_id = jointNameToId.get('right_shoulder')
    elbow_right_id = jointNameToId.get('right_elbow')

    if shoulder_left_id is None or elbow_left_id is None or shoulder_right_id is None or elbow_right_id is None:
        print("Error:  Could not find all required shoulder and elbow joints.  Make sure your URDF has joints named 'left_shoulder', 'left_elbow', 'right_shoulder', and 'right_elbow'.")
        p.disconnect()
        return  # Exit the function if the joints are not found.

    try:
        while True:
            t = time.time()
            angle = 0.5 * n.sin(2 * t)

            p.setJointMotorControl2(robotId, shoulder_left_id, p.POSITION_CONTROL, targetPosition=angle)
            p.setJointMotorControl2(robotId, elbow_left_id, p.POSITION_CONTROL, targetPosition=-angle)
            p.setJointMotorControl2(robotId, shoulder_right_id, p.POSITION_CONTROL, targetPosition=angle)
            p.setJointMotorControl2(robotId, elbow_right_id, p.POSITION_CONTROL, targetPosition=-angle)

            p.stepSimulation()
            time.sleep(1. / 240.)

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        p.disconnect()

if __name__ == '__main__':
    # Example usage:
    urdf_file = "humanoid_robot.urdf"  # Or your specific URDF file
    initial_pose = {
        "left_shoulder": 0.0,
        "left_elbow": 0.0,
        "right_shoulder": 0.0,
        "right_elbow": 0.0,
        # Add other joints and angles as needed for your robot's initial pose
    }
    setup_and_control_robot(urdf_path=urdf_file, initial_joint_angles=initial_pose)
