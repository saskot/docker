import pybullet as p
import pybullet_data
import numpy as np
import time

def numerical_ik(robot_id, end_effector_index, target_pos, joint_indices, max_iters=100, treshold=1e-3, alpha=0.1, damping=0.01):
    for _ in range(max_iters):
        joint_states = [p.getJointState(robot_id, i)[0] for i in joint_indices]

        link_state = p.getLinkState(robot_id, end_effector_index, computeForwardKinematics=True)
        curent_pos = np.array(link_state[4])

        error = np.array(target_pos) - curent_pos
        if np.linalg.norm(error) < treshold:
            break

        zero_vec = [0.0] * len(joint_indices)
        J_lin, J_ang = p.calculateJacobian(
            robot_id,
            end_effector_index,
            localPosition=[0, 0, 0],
            objPositions=joint_states,
            objVelocities=zero_vec,
            objAccelerations=zero_vec,
        )

        J = np.array(J_lin)
        JT = J.T
        d_theta = alpha * JT @ np.linalg.inv(J @ JT + damping**2 * np.eye(3)) @ error

        for i, idx in enumerate(joint_indices):
            p.resetJointState(robot_id, idx, joint_states[i] + d_theta[i])
            time.sleep(1 / 100)

        p.stepSimulation()


if __name__ == "__main__":
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    p.loadURDF("plane.urdf")
    kuka_id = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)

    end_effector_index = 6
    target_position = [0.7, 0.0, 0.5]
    movable_joints = [
        i for i in range(p.getNumJoints(kuka_id))
        if p.getJointInfo(kuka_id, i)[2] != p.JOINT_FIXED
    ]

    numerical_ik(kuka_id, end_effector_index, target_position, movable_joints)

    final_pos = p.getLinkState(kuka_id, end_effector_index)[4]
    print("target:", np.round(target_position, 3))
    print("achieved:", np.round(final_pos, 3))
    input("press enter to disconnect")
    p.disconnect()