import control as ctrl
import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt


def calculate_L_COM(m1: float, m2: float, 
                    L1: float, L2: float) -> float:
    """
    This function calculates the Center of Mass Length
    """
    L = (L2 / 2) + ((L1 + L2)*m1)/(2 * (m1 + m2))
    return L

def calculate_inertia(m1: float, m2: float,
                      L1: float, L2: float) -> float:
    """
    This function calculate the inertia of the whole body
    """
    I = m1 * ((L1 / 2) + L2)* ((L1 / 2) + L2) + (m2 * L2 * L2) / 12
    return I


# Define the system parameters
m = 513.3 * 10e-3 # mass of body part [kg]
m2 = 0 * 10e-3
m1 = m - m2
m_w = 7.2 * 10e-3 # mass of the wheel part [kg]
R = 16.0 * 10e-3 # radius of the wheel [m]
L1 = 40 * 10e-3 # height of the head [m]
L2 = 60 * 10e-3 # height of the head [m]
L = calculate_L_COM(m1=m1, m2=m2, L1=L1, L2=L2) # length between the Center of Mass (COM) and the center of the wheel
tau_0 = 1 # applied torque [kg*m^2]
I = calculate_inertia(m1=m1, m2=m2, L1=L1, L2=L2) # inertia of body part [kg*m^2]
I_w = 779.2 * 10e-9 # inertia of the wheel part [kg*m^2]
g = 9.8 # gravitz [m/s^2]
beta_r = 0.01 # rolling damping ratio [N*m/(rad/s)]
beta_m = 0.01 # bearing damping ratio [N*m/(rad/s)] 

# Define the matrices
E = np.array(
    [
        [I_w + (m + m_w) * R * R, m * R * L],
        [m * R * L, I + (m * m) * L]
    ]
)
F = np.array(
    [
        [beta_r + beta_m, -beta_m],
        [-beta_m, beta_m]
    ]
)
G = np.array(
    [
        [0],
        [-m * g * L]
    ]
)
H = np.array(
    [
        [1],
        [-1]
    ]
)

# Define state-space representation
a11_a21 = np.zeros((2, 1))
a12_a22 = np.zeros((2, 1))
a13_a14_a23_a24 = np.identity(2)
a31_a41 = np.zeros((2, 1))
a32_a42 = -1 * inv(E) @ G
a33_a34_a43_a44 = -1 * inv(E) @ F

A = np.block(
    [
        [a11_a21, a12_a22, a13_a14_a23_a24],
        [a31_a41, a32_a42, a33_a34_a43_a44]
    ]
)

b11_b21 = np.zeros((2, 1))
b31_b41 = -1 * inv(E) @ H
B = np.block(
    [
        [b11_b21],
        [b31_b41]
    ]
)

C = np.array(
    [
        [R, 0, 0, 0],
        [0, 1, 0, 0]
    ]
)

D = 0



if __name__ == "__main__":
    # define state-space representation
    sys_ss = ctrl.ss(A, B, C, D)

    # convert to transfer function
    sys_tf = ctrl.ss2tf(sys_ss)

    # Define the PID controller
    Kp = 10   # Proportional gain
    Ki = 5    # Integral gain
    Kd = 2    # Derivative gain
    pid = ctrl.tf([Kd, Kp, Ki], [1, 0])

    # Connect the PID Controller to the System
    closed_loop = ctrl.feedback(pid * sys_tf)


    # plot response of system
    time, response = ctrl.step_response(closed_loop)

    plt.plot(time, response)
    plt.xlabel("Time (s)")
    plt.ylabel("Response")
    plt.title("Step Response with PID Control")
    plt.grid()
    plt.show()

