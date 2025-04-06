## Capstone_Robotic_Manipulation

# Best simulation

# Description:
This README is for both best simulation folders namely Avoid_self_collision and 
No_joint_limits.

The python file `Milestone3.py` in the respective folders consistes of all 
three Milestones combined.

Cut and paste this command in the command line to generate the csv file:
(You must be in the same directory as the python file.)
    +  `python3 -m Milestone3.py`

1) No_joint_limits:
    In this folder there is two seperate simulations namely No_joint_limit and 
    Self_collision. Both of these simulations has no joint limits and is free
    to rotate.
        1.1) No_joint_limit:
            Type of controller: Feedforward with PI - Kp = 5.0, Ki = 0.01
            Initial configiration of end-effector relative to world:
                Tse_initial = np.array([[ 0, 0, 1,  0 ],
                                        [ 0, 1, 0,  0 ],
                                        [-1, 0, 0, 0.5],
                                        [ 0, 0, 0,  1 ]])
            Results: (Refers to video and Xerr graph)
                From observation of the graph and the simulation it is evident
                that all 6 errors converges to zero in 1[s] and then follows the
                desired trajectory perfectly for 90% of the time. There is no
                overshoot. The video shows the robot quickly moving back to its
                reference trajectory as a result of the PI gains and then
                follows the desired trajectoty to pick up the block and place
                it in the desired location.
            
        1.2) Self_collision:
            In this simulation the robot arm was forced to have a lot of self
            collisions in order to have a base case to test the
            testJointLimits() function on to ensure that it is working properly.
            The self collisions was ensured by lowering the initial height
            (z coordinate) of the end-effector from 0.5 to 0.25
        
            Type of controller: Feedforward with PI - Kp = 5.0, Ki = 0.01
            
            Initial configiration of end-effector relative to world:
            (Uncomment line 132-135)
                Tse_initial = np.array([[ 0, 0, 1,  0 ],
                                        [ 0, 1, 0,  0 ],
                                        [-1, 0, 0, 0.25],
                                        [ 0, 0, 0,  1 ]])
            Results: (Refers to video and Xerr graph)
                From observation of the graph and the simulation it is evident
                that all 6 errors converges to zero in 1[s] and then follows the
                desired trajectory perfectly for 90% of the time. There is no
                overshoot. The video shows the robot quickly moving back to its
                reference trajectory as a result of the PI gains and then
                follows the desired trajectoty to pick up the block and place
                it in the desired location. The arm can be seen to rotate freely
                and experience self collision this will be fixed with the
                testJointLimits() function in the folder Avoid_self_collision.
                
  
2) Avoid_self_collision:
    In this simulation the robot arm cannot collide with itself as a
    testJointLimits() function was implemented that stops the joint from
    rotating past predetermined joint limits.
    
    Joint limits for arm joints:
        arm1 = np.array([-0.8, 0.8])
        arm2 = np.array([-0.1, -1.8])
        arm3 = np.array([-0.5, -1.8])
        arm4 = np.array([1.8, -1.8])

    Type of controller: Feedforward with PI - Kp = 1.0, Ki = 0.01
    
    Initial configiration of end-effector relative to world:
    (Uncomment line 132-135)
        Tse_initial = np.array([[ 0, 0, 1,  0 ],
                                [ 0, 1, 0,  0 ],
                                [-1, 0, 0, 0.25],
                                [ 0, 0, 0,  1 ]])
    Results: (Refers to video and Xerr graph)
        From observation of the graph and the simulation it is evident
        that all 6 errors converges to zero in 4[s]. Pitch, Yaw and Roll error
        stays zero, but X, Y and Z error stays close to zero but not exactly
        zero. This can be a result of the decrease in the workspace of the robot
        arm. Thus it is not always possible for the arm to follow the desired
        trajectory. The video shows the robot quickly moving back to its
        reference trajectory as a result of the PI gains and then
        relatively follows the desired trajectoty to pick up the block and place
        it in the desired location. From the video it is evident that the robot
        arm does not colide with itself and thus the self collision avoidance
        function works properly. This video should be compared to the video
        Self_collision in the forlder No_joint_limits. From this it is evident
        that the self collision avoidance works.
        
    HOW I IMPLEMENTED JOINT LIMIT CONTROL:
        I used the arm joint sliders in scene 3 to find the resonably range of
        the joints considering the mechanical stops on on the robot arm. A
        function called testJointLimits() was implemented. This determines if
        the next state's joints will violate the joint limits. If one of the
        joints will violate the limit then the jacobian column that corresponds
        with that joint is set to 0 and the control velocities are recalculated
        without changing that joints angle. After this function was implemented
        and working the joint limits were decreased to ensure that self
        collions is not possible.
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
    

