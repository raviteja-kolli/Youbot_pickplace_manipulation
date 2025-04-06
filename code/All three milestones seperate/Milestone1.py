"""
Cut and paste this command in the command line to generate the csv file: 
(You must be in the same directory as the python file.)
    +  python3 -m Milestone1.py

Non self collision - Make jacobian 0 when close to self collision
page 150 in mr book fro Part 1
"""
import modern_robotics as mr
import numpy as np


def append_to_csv(CurrentState):
    """Appends CurrentState to an existing csv file."""
    with open("Milestone1.csv",'a') as csvfile:
        np.savetxt(csvfile, CurrentState, delimiter = ",")


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
        # print(f"\n wheel_velocity[i] = {wheel_velocity[i]}, new_wheel_angles[i] = {new_wheel_angles[i]}, old_wheel_angles[i] = {old_wheel_angles[i]}, dt = {dt}, wheel_velocity[i]*dt = {wheel_velocity[i]*dt}")
        # print(f"\n old_wheel_angles[i] + wheel_velocity[i]*dt = {old_wheel_angles[i] + wheel_velocity[i]*dt}")
        # print(f"\n new_wheel_angles = {new_wheel_angles}")
    # Calculate new chassis configuration
    Vb = Fo@wheel_velocity
    # print(f"\n Vb = {Vb}")
    # Tb = np.exp(Vb)
    # Calculate q in the {b} frame
    if Vb[0] == 0: # Omega_b_x = 0
        Delta_qb = np.array([0, Vb[1],Vb[2]])
    else:
        Delta_qb = np.array([Vb[0],
                            (Vb[1]*np.sin(Vb[0]) + Vb[2]*(np.cos(Vb[0]-1)))/Vb[0],
                            (Vb[2]*np.sin(Vb[0]) + Vb[1]*(1-np.cos(Vb[0])))/Vb[0]])
    # Change q from the {b} frame to the {s} frame
    phi_k = CurrentState[0]
    Rx = np.array([[1,       0      ,       0      ],
                    [0, np.cos(phi_k),-np.sin(phi_k)],
                    [0, np.sin(phi_k), np.cos(phi_k)]])
    Delta_q = Rx@Delta_qb
    # print(f"Delta_qb = {Delta_qb}")
    # print(f"Delta_q = {Delta_q}")
    # print(f"qk = {qk}")
    new_chassis_cofig = qk + Delta_q*dt
    # print(f"new_chassis_cofig = {new_chassis_cofig}")

    # next_state = np.array([3 x chassis config, 5 x arm config, 4 x wheel config, 0/1 for gripper])
    # next_state = np.concatenate((new_chassis_cofig,new_joint_angles,new_wheel_angles,0), axis = None)
    next_state = np.hstack((new_chassis_cofig,new_joint_angles,new_wheel_angles,0))
    print(f"\n next_state = {next_state}")

    return next_state



# Joint and wheel velocity array
joint_velocity = np.array([0,0,0,0,0])
# joint_velocity = np.array([1,1,1,1,1])
wheel_velocity = np.array([10,10,10,10]) # Drive forward
# wheel_velocity = np.array([-10,10,-10,10]) # Slide sideways
# wheel_velocity = np.array([-10,10,10,-10]) # Spin in place
# Max velocity limit
VelocityLimit = 100 # m/s
# ControlVelocities
# ControlVelocities = np.concatenate((wheel_velocity,joint_velocity),axis = None)
ControlVelocities = np.hstack((wheel_velocity,joint_velocity))
# Start State
Start_state = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
CurrentState = Start_state
PrevState = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
All_states = Start_state

# Robot dimensions
l = float(0.47/2) # Wheel to centre of body in length
w = float(0.3/2) # Wheel to centre of body in width
r = 0.0475 # Wheel radius
Fo = (r/4)*np.array([[-1.0/(l+w), 1.0/(l+w), 1.0/(l+w), -1.0/(l+w)],
                    [    1.0    ,     1.0  ,     1.0  ,     1.0   ],
                    [   -1.0    ,     1.0  ,    -1.0  ,     1.0   ]])

# Use for all trajectorys
dt = 0.01

# Create csv file
# np.savetxt("Milestone1.csv", Start_state, delimiter = ",")

for i in range(100):
    CurrentState = NextState(CurrentState, ControlVelocities, dt, VelocityLimit)
    All_states = np.vstack((All_states,CurrentState))

append_to_csv(All_states)