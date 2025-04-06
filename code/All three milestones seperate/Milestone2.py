"""
Cut and paste this command in the command line to generate the csv file: 
(You must be in the same directory as the python file.)
    +  python3 -m Milestone2.py
"""
import modern_robotics as mr
import numpy as np

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
# Gripper initial, End effector initial configuration
Tse_initial = np.array([[ 0, 0, 1,  0 ],
                        [ 0, 1, 0,  0 ],
                        [-1, 0, 0, 0.5],
                        [ 0, 0, 0,  1 ]])

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

# Append to csv file
append_to_csv(trajectory_mat)
