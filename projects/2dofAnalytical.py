import numpy as np 
import matplotlib.pyplot as plt 
import time

l1=1
l2=1

def inverse_kinematics(x, y):
    cos_theta2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    
    if np.abs(cos_theta2) > 1:
        raise ValueError("Target is out of reach")
    
    solutions = []
    theta2_candidates = [np.arccos(cos_theta2), -np.arccos(cos_theta2)]
    
    for theta2 in theta2_candidates:
        k1 = l1 + l2 * np.cos(theta2)
        k2 = l2 * np.sin(theta2)
        theta1 = np.atan2(y, x) - np.atan2(k2, k1)
        solutions.append((theta1, theta2))
    return solutions

def forward_kinematics(theta1, theta2):
    x1 = l1 * np.cos(theta1)
    y1 = l1 * np.sin(theta1)
    x2 = x1 + l2 * np.cos(theta1 + theta2)
    y2 = y1 + l2 * np.sin(theta1 + theta2)
    return (0,x1,x2), (0,y1,y2)

def plot_arm(x,y):
    try:
        solutions = inverse_kinematics(x, y)
        for index, (theta1, theta2) in enumerate(solutions):
            x_coords, y_coords = forward_kinematics(theta1, theta2)
            plt.plot(x_coords, y_coords, marker='o', label=f'Solution {index+1}')
            print(theta1, theta2)
            
        plt.plot(x, y, 'rx',markersize =10, label='Target')
        plt.xlim(-2.5, 2.5)
        plt.ylim(-2.5, 2.5)
        plt.gca().set_aspect('equal')
        plt.grid(True)
        plt.title(f"Analytical IK for 2-DOF Arm to reach ({x:.2f}, {y:.2f})")
        plt.show()
    except ValueError as e:
        print(f"Error: {e} {x:.2f}, {y:.2f} is out of reach.")

if __name__ == "__main__":
    plot_arm(1.5, 1.5)
    plot_arm(0.5, 1.5)
    plot_arm(0, 2) 
    plot_arm(3, 1.5)