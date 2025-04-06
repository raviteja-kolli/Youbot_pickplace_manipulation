"""
Cut and paste this command in the command line to generate the csv file: 
(You must be in the same directory as the python file.)
    +  python3 -m Milestone3.py
"""
import modern_robotics as mr
import numpy as np
import matplotlib.pyplot as plt



"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
                                            MILESTONE 1
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

def NextState(CurrentState, ControlVelocities, dt, VelocityLimit):
    """
    Calculate next state using a simple first-order euler step

    Args:
        + CurrentState -
        + ControlVelocities -
        + dt -
        + VelocityLimit -

    Return:
        + next_state -
    """
    # Current chassis state
    qk = CurrentState[0:3]
    # Old and New joint angle array
    old_joint_angles = CurrentState[3:8]
    new_joint_angles = np.array([0.0,0.0,0.0,0.0,0.0])
    # Old and New wheel angle array
    old_wheel_angles = CurrentState[8:12]
    new_wheel_angles = np.array([0.0,0.0,0.0,0.0])

    # Velocities
    joint_velocity = ControlVelocities[4:]
    wheel_velocity = ControlVelocities[0:4]

    # Constrain velocities to Max velocity limit
    for x in range(len(ControlVelocities)):
        if ControlVelocities[x] > VelocityLimit:
            ControlVelocities[x] = VelocityLimit
        elif ControlVelocities[x] < -VelocityLimit:
            ControlVelocities[x] = -VelocityLimit

    # Calculate new joint angles
    for i in range(len(old_joint_angles)):
        new_joint_angles[i] = old_joint_angles[i] + joint_velocity[i]*dt

    # Calculate new wheel angles
    for i in range(len(old_wheel_angles)):
        new_wheel_angles[i] = old_wheel_angles[i] + wheel_velocity[i]*dt
    # Calculate new chassis configuration
    Vb = Fo@wheel_velocity

    # Calculate q in the {b} frame
    if Vb[0] == 0: # Omega_b_x = 0
        Delta_qb = np.array([0, Vb[1],Vb[2]])
    else:
        Delta_qb = np.array([Vb[0],
                            (Vb[1]*np.sin(Vb[0]) + Vb[2]*(np.cos(Vb[0])-1))/Vb[0],
                            (Vb[2]*np.sin(Vb[0]) + Vb[1]*(1-np.cos(Vb[0])))/Vb[0]])
    # Change q from the {b} frame to the {s} frame
    phi_k = CurrentState[0]
    Rx = np.array([[1,       0      ,       0      ],
                    [0, np.cos(phi_k),-np.sin(phi_k)],
                    [0, np.sin(phi_k), np.cos(phi_k)]])
    Delta_q = Rx@Delta_qb
    new_chassis_cofig = qk + Delta_q*dt
    next_state = np.hstack((new_chassis_cofig,new_joint_angles,new_wheel_angles,0))

    return next_state

# Robot dimensions
l = float(0.47/2) # Wheel to centre of body in length
w = float(0.3/2) # Wheel to centre of body in width
r = 0.0475 # Wheel radius
Fo = (r/4)*np.array([[-1.0/(l+w), 1.0/(l+w), 1.0/(l+w), -1.0/(l+w)],
                    [    1.0    ,     1.0  ,     1.0  ,     1.0   ],
                    [   -1.0    ,     1.0  ,    -1.0  ,     1.0   ]])



"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
                                            MILESTONE 2
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

def Milestone2():
    import modern_robotics as mr
    import numpy as np

    # Total desired trajectory
    Desired_trajectory = np.array([])

    # Gripper geometry
    d1 = 7.0/100.0 # Space between open grippers
    d2 = 3.5/100.0 # Lenght of fingers
    d3 = 4.3/100.0 # Lenght from gripper frame to back of grippers

    # Use for all trajectorys
    dt = 0.01
    method = 5
    k = 1
    Tf_vec = np.array([2,1,1,1,3,1,1,1]) # Time for all trajectories
    gripper_vec = np.array([0, 0, 1, 1, 1, 1, 0, 0]) # Gripper state for each trajectory
    traj_select_vec = np.array([1, 0, 0, 0, 1, 0, 0, 0]) # Select Cartesian or ScrewTrajectory

    # Cube start location
    Cube_i = np.array([1,0,0.025])
    # Cube goal desired location
    Cube_g = np.array([0,-1,0.025])

    # Cube initial configuration
    Tsc_initial = np.array([[1, 0, 0,   1  ],
                            [0, 1, 0,   0  ],
                            [0, 0, 1, 0.025],
                            [0, 0, 0,   1  ]])
    # Cube final configuration
    Tsc_goal = np.array([[ 0, 1, 0,   0  ],
                        [-1, 0, 0,  -1  ],
                        [ 0, 0, 1, 0.025],
                        [ 0, 0, 0,   1  ]])
    """ New Block configuration """
    Tse_initial = np.array([[ 0, 0, 1,  0 ],
                            [ 0, 1, 0,  0 ],
                            [-1, 0, 0, 0.5],
                            [ 0, 0, 0,  1 ]])
    """ Forced self collision """
    # Tse_initial = np.array([[ 0, 0, 1,  0 ],
    #                         [ 0, 1, 0,  0 ],
    #                         [-1, 0, 0, 0.25],
    #                         [ 0, 0, 0,  1 ]])




    angle_grasp = 1.785398 # Angle for end effector

    # Standoff before and after cube grab relative to cube
    # Add d2/2 to have cube in centre of gripper
    Tce_standoff = np.array([[ np.cos(angle_grasp), 0,  np.sin(angle_grasp),   d2/2.0  ],
                            [          0,          1,            0,            0    ],
                            [-np.sin(angle_grasp), 0,  np.cos(angle_grasp),   0.3   ],
                            [          0,          0,             0,           1    ]])

    # End effector configuration relative to cube when grasp
    # Add d2/2 to have cube in centre of gripper
    Tce_grasp = np.array([[ np.cos(angle_grasp), 0,  np.sin(angle_grasp),  d2/2.0 ],
                        [          0,          1,            0,             0   ],
                        [-np.sin(angle_grasp), 0,  np.cos(angle_grasp),     0   ],
                        [          0,          0,             0,            1   ]])


    def Traj_specs(Tf):
        """Function to calculate N and create a trajectory_mat from the Tf (time)."""
        N = int((Tf*k/dt)+1)
        trajectory_mat = np.zeros((N,13))
        return N, trajectory_mat

    def append_to_csv(trajectory_mat):
        """Appends trajectory_mat to an existing csv file."""
        with open("Milestone2.csv",'a') as csvfile:
            np.savetxt(csvfile, trajectory_mat, delimiter = ",")


    def TrajectoryGenerator(Xstart, Xend, Tf, N, method, trajectory_mat, traj_select, grasp):
        """
        Trajectory generator function
        """
        if traj_select == 0:
            traj = mr.CartesianTrajectory(Xstart, Xend, Tf, N, method)
        else:
            traj = mr.ScrewTrajectory(Xstart, Xend, Tf, N, method)

        for i in range(N):
            m = 0
            for j in range(3):
                for k in range(3): # extract r11, r12, r13 - r21, r22, r23, - r31, r32, r33
                    l = k+m
                    trajectory_mat[i,l] = traj[i][j,k]
                f = 9+j
                trajectory_mat[i,f] = traj[i][j,3] # extract px, py, pz
                m += 3 # Scale by one row [3 elements]
            trajectory_mat[i,12] = grasp # Set Gripper

        return trajectory_mat


    """
    Traj 1 - Initial to standoff initial
    """
    Tf = 5 # Amount of time for motion
    N, trajectory_mat = Traj_specs(Tf)
    grasp = gripper_vec[0] # Open gripper

    Xstart = Tse_initial
    Xend = Tsc_initial@Tce_standoff
    traj_select = traj_select_vec[0] # Select screw trajectory

    trajectory_mat = TrajectoryGenerator(Xstart, Xend, Tf, N, method, trajectory_mat, traj_select,grasp)
    Desired_trajectory = trajectory_mat

    # Create csv file
    np.savetxt("Milestone2.csv", trajectory_mat, delimiter = ",")


    """
    Traj 2 - standoff initial to grasp initial
    """
    Tf = Tf_vec[1] # Amount of time for motion
    N, trajectory_mat = Traj_specs(Tf)
    grasp = gripper_vec[1] # Open gripper

    Xstart = Tsc_initial@Tce_standoff
    Xend = Tsc_initial@Tce_grasp
    traj_select = traj_select_vec[1] # Cartesian trajectory

    trajectory_mat = TrajectoryGenerator(Xstart, Xend, Tf, N, method, trajectory_mat, traj_select, grasp)
    Desired_trajectory = np.vstack((Desired_trajectory, trajectory_mat))

    # Append to csv file
    append_to_csv(trajectory_mat)


    """
    Traj 3 - Grasp initial
    """
    Tf = Tf_vec[2] # Wai for 1sec to grab block
    N, trajectory_mat = Traj_specs(Tf)
    grasp = gripper_vec[2] # Close gripper

    Xstart = Tsc_initial@Tce_grasp
    Xend = Tsc_initial@Tce_grasp
    traj_select = traj_select_vec[2] # Cartesian trajectory

    trajectory_mat = TrajectoryGenerator(Xstart, Xend, Tf, N, method, trajectory_mat, traj_select, grasp)
    Desired_trajectory = np.vstack((Desired_trajectory, trajectory_mat))

    # Append to csv file
    append_to_csv(trajectory_mat)


    """
    Traj 4 - Grasp initial to standoff initial
    """
    Tf = Tf_vec[3] # Amount of time for motion
    N, trajectory_mat = Traj_specs(Tf)
    grasp = gripper_vec[3] # Close gripper

    Xstart = Tsc_initial@Tce_grasp
    Xend = Tsc_initial@Tce_standoff
    traj_select = traj_select_vec[3] # Cartesian trajectory

    trajectory_mat = TrajectoryGenerator(Xstart, Xend, Tf, N, method, trajectory_mat, traj_select, grasp)
    Desired_trajectory = np.vstack((Desired_trajectory, trajectory_mat))

    # Append to csv file
    append_to_csv(trajectory_mat)


    """
    Traj 5 - Standoff initial to standoff goal
    """
    Tf = Tf_vec[4] # Amount of time for motion
    N, trajectory_mat = Traj_specs(Tf)
    grasp = gripper_vec[4] # Close gripper

    Xstart = Tsc_initial@Tce_standoff
    Xend = Tsc_goal@Tce_standoff
    traj_select = traj_select_vec[4] # Screw trajectory

    trajectory_mat = TrajectoryGenerator(Xstart, Xend, Tf, N, method, trajectory_mat, traj_select, grasp)
    Desired_trajectory = np.vstack((Desired_trajectory, trajectory_mat))

    # Append to csv file
    append_to_csv(trajectory_mat)


    """
    Traj 6 - Standoff goal to grasp goal
    """
    Tf = Tf_vec[5] # Amount of time for motion
    N, trajectory_mat = Traj_specs(Tf)
    grasp = gripper_vec[5] # Close gripper

    Xstart = Tsc_goal@Tce_standoff
    Xend = Tsc_goal@Tce_grasp
    traj_select = traj_select_vec[5] # Cartesian trajectory

    trajectory_mat = TrajectoryGenerator(Xstart, Xend, Tf, N, method, trajectory_mat, traj_select, grasp)
    Desired_trajectory = np.vstack((Desired_trajectory, trajectory_mat))

    # Append to csv file
    append_to_csv(trajectory_mat)



    """
    Traj 7 - Grasp goal - Release block
    """
    Tf = Tf_vec[6] # Amount of time for motion
    N, trajectory_mat = Traj_specs(Tf)
    grasp = gripper_vec[6] # Close gripper

    Xstart = Tsc_goal@Tce_grasp
    Xend = Tsc_goal@Tce_grasp
    traj_select = traj_select_vec[6] # Cartesian trajectory

    trajectory_mat = TrajectoryGenerator(Xstart, Xend, Tf, N, method, trajectory_mat, traj_select, grasp)
    Desired_trajectory = np.vstack((Desired_trajectory, trajectory_mat))

    # Append to csv file
    append_to_csv(trajectory_mat)


    """
    Traj 8 - Grasp goal to Standoff goal
    """
    Tf = Tf_vec[7] # Amount of time for motion
    N, trajectory_mat = Traj_specs(Tf)
    grasp = gripper_vec[7] # Close gripper

    Xstart = Tsc_goal@Tce_grasp
    Xend = Tsc_goal@Tce_standoff
    traj_select = traj_select_vec[7] # Cartesian trajectory

    trajectory_mat = TrajectoryGenerator(Xstart, Xend, Tf, N, method, trajectory_mat, traj_select, grasp)
    Desired_trajectory = np.vstack((Desired_trajectory, trajectory_mat))

    # Append to csv file
    append_to_csv(trajectory_mat)

    return Desired_trajectory



"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
                                            MILESTONE 3 
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
def append_to_csv(CurrentState):
    """Appends CurrentState to an existing csv file."""
    with open("Milestone3.csv",'a') as csvfile:
        np.savetxt(csvfile, CurrentState, delimiter = ",")




def FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, Xerr_Total, Xerr):

    Kp = Kp*np.identity(6)
    Ki = Ki*np.identity(6)

    Xerr = mr.se3ToVec(mr.MatrixLog6(mr.TransInv(X) @ Xd))

    Vd =  mr.se3ToVec((1/dt) * mr.MatrixLog6((mr.TransInv(Xd) @ Xd_next)))

    """ Different controllers """
    # # Feedforward control and PI
    Ve = mr.Adjoint(mr.TransInv(X) @ Xd) @ Vd + Kp@Xerr + Ki@Xerr_Total
    # # Feedforward control and P
    # Ve = mr.Adjoint(mr.TransInv(X) @ Xd) @ Vd + Kp@Xerr
    # # Feedforward control
    # Ve = mr.Adjoint(mr.TransInv(X) @ Xd) @ Vd
    # P control
    # Ve = Kp@Xerr
    # # PI control
    # Ve = Kp@Xerr + Ki@Xerr_Total

    return Ve, Xerr, Xerr_Total


# Start configuration
Start_state = np.array([0.3, 0, 0.2, 0, -0.2, -0.6, -1.578, 0, 0, 0, 0, 0, 0])
curr_config = np.array([0.3, 0, 0.2, 0, -0.2, -0.6, -1.578, 0])

# PID Gains
Kp = 5
Ki = 0.01

# Time step
dt = 0.01

# Robot dimensions
l = float(0.47/2) # Wheel to centre of body in length
w = float(0.3/2) # Wheel to centre of body in width
r = 0.0475 # Wheel radius

F6 = (r/4)*np.array([[0, 0, 0, 0],
                    [0, 0, 0, 0],
                    [-1.0/(l+w), 1.0/(l+w), 1.0/(l+w), -1.0/(l+w)],
                    [    1.0    ,     1.0  ,     1.0  ,     1.0   ],
                    [   -1.0    ,     1.0  ,    -1.0  ,     1.0   ],
                    [0, 0, 0, 0]])


# Fixed offset from chassis frame {b} to base of arm frame {0}
Tb0 = np.array([[1, 0, 0, 0.1662],
                [0, 1, 0,    0  ],
                [0, 0, 1, 0.0026],
                [0, 0, 0,    1  ]])
# Home configuration. End effector frame {e} relative to the arm base frame {0}
M0e = np.array([[1, 0, 0,  0.033],
                [0, 1, 0,    0  ],
                [0, 0, 1, 0.6546],
                [0, 0, 0,    1  ]])
# Arm at home configuration, the screw axes in {b} frame for the five joints are expressed
# in the end-effector frame {e} as
Blist = np.array([[  0  ,    0,     0,       0,   0],
                [  0  ,   -1,    -1,      -1,   0],
                [  1  ,    0,     0,       0,   1],
                [  0  ,-0.5076,-0.3526,-0.2176, 0],
                [0.033,    0,     0,       0,   0],
                [  0,      0,     0,       0,   0]])

# Extraxt joint angles
joint_angles = curr_config[3:]
# Calculate transformation matrix of end-effector {e} relative to the base of the arm {0}
T0e = mr.FKinBody(M0e,Blist,joint_angles)

# Extraxt robot base angle and x,yz coordinates
phi = curr_config[0]
x = curr_config[1]
y = curr_config[2]
z = 0.0963
# Chassis frame {b} relative to world frame {s}
Tsb = np.array([[np.cos(phi), -np.sin(phi), 0, x],
                [np.sin(phi),  np.cos(phi), 0, y],
                [    0      ,       0     , 1, z],
                [    0      ,       0     , 0, 1]])

# Tse - Current configuration of robot end-effector {e} relative to world frame {s}
X = Tsb @ Tb0 @ T0e

# Jacobian of arm
J_arm = mr.JacobianBody(Blist, joint_angles)
J_base = mr.Adjoint(np.linalg.pinv(T0e) @ np.linalg.pinv(Tb0)) @ F6
Je = np.hstack((J_base, J_arm))


""" Milestone2 Reference Trajectory. """
# Creat desired trajectory
Desired_trajectory = Milestone2()
print(f"\n\n Calculated desired trajectory")

def TransMat(R):
    return np.array([[R[0], R[1], R[2], R[9]],
                     [R[3], R[4], R[5], R[10]],
                     [R[6], R[7], R[8], R[11]],
                     [ 0  ,  0  ,   0 ,   1  ]])

# Max velocity
VelocityLimit = 40 #*10 # 100
# 
# Gripper initial, End effector initial configuration
Tse_initial = np.array([[ 0, 0, 1,  0 ],
                        [ 0, 1, 0,  0 ],
                        [-1, 0, 0, 0.5],
                        [ 0, 0, 0,  1 ]])

CurrentState = Start_state
Xtotal = Start_state

Xerr_Total = np.zeros((6))
Xerr = np.zeros((6))

Flag = 0
Xerr1 = []
Xerr2 = []
Xerr3 = []
Xerr4 = []
Xerr5 = []
Xerr6 = []

for t in range(0, Desired_trajectory.shape[0]-1):
    # Milestone2 - Calculate desired trajectory and next desired trajectory
    Xd = TransMat(Desired_trajectory[t])
    Xd_next = TransMat(Desired_trajectory[t+1])

    curr_config = CurrentState[:12]
    # Extraxt joint angles
    joint_angles = curr_config[3:8]

    # Calculate transformation matrix of end-effector {e} relative to the base of the arm {0}
    T0e = mr.FKinBody(M0e,Blist,joint_angles)

    # Extraxt robot base angle and x,yz coordinates
    phi = curr_config[0]
    x = curr_config[1]
    y = curr_config[2]
    z = 0.0963
    # Chassis frame {b} relative to world frame {s}
    Tsb = np.array([[np.cos(phi), -np.sin(phi), 0, x],
                    [np.sin(phi),  np.cos(phi), 0, y],
                    [    0      ,       0     , 1, z],
                    [    0      ,       0     , 0, 1]])


    # Tse - Current configuration of robot end-effector {e} relative to world frame {s}
    X = Tsb @ Tb0 @ T0e


    # Jacobian of arm
    J_arm = mr.JacobianBody(Blist, joint_angles)
    J_base = mr.Adjoint(np.linalg.pinv(T0e) @ np.linalg.pinv(Tb0)) @ F6
    Je = np.hstack((J_base, J_arm))

    # Milestone 3 - PID controller 
    Ve, Xerr, Xerr_Total= FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, Xerr, Xerr_Total)
    Xerr_Total += Xerr*dt

    # Command velocities
    u_theta_Dot = np.linalg.pinv(Je)@Ve
    ControlVelocities = u_theta_Dot

    # Milestone 1 - Get next state
    CurrentState = NextState(CurrentState, ControlVelocities, dt, VelocityLimit)

    stack = CurrentState
    stack[-1] = Desired_trajectory[t][-1]
    Xtotal = np.vstack((Xtotal, stack))

    if Flag == 0:
        plot_Xerr = Xerr
        Flag = 1
    else:
        plot_Xerr = np.vstack((plot_Xerr, Xerr))

    Xerr1.append(Xerr[0])
    Xerr2.append(Xerr[1])
    Xerr3.append(Xerr[2])
    Xerr4.append(Xerr[3])
    Xerr5.append(Xerr[4])
    Xerr6.append(Xerr[5])

print(" Done with simulation ")
print(" Plotting ")
print(" --- Close plot for .csv to save ---")

plt.plot( Xerr1, label = 'Roll Error')
plt.plot( Xerr2, label = 'Pitch Error')
plt.plot( Xerr3, label = 'Yaw Error')
plt.plot( Xerr4, label = 'X Error')
plt.plot( Xerr5, label = 'Y Error')
plt.plot( Xerr6, label = 'Z Error')
plt.ylabel('Error [rad & m]')
plt.xlabel('Time [ms]')
plt.title('[Xerr] vs Time')
plt.legend()
plt.show()

print(" Saving to no_joint_limits.csv and Xerr.csv... ")
# Create csv file
np.savetxt("no_joint_limits.csv", Xtotal, delimiter = ",")
np.savetxt("Xerr.csv", Xtotal, delimiter = ",")

print(" DONE! ")
print("\n\n\n\n")
