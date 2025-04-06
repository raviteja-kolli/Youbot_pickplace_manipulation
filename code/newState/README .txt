## Capstone_Robotic_Manipulation

# Next state simulation

# Description:
This README is for both the case where the block start and end configuration was placed at a new position

The python file `Milestone3.py` in the respective folders consistes of all 
three Milestones combined.

Cut and paste this command in the command line to generate the csv file:
(You must be in the same directory as the python file.)
    +  `python3 -m Milestone3.py`

1) No_joint_limit:
    Type of controller: Feedforward with PI - Kp = 5.0, Ki = 0.01
    New Block configuration:
        # Cube initial configuration
        - The orientation is the same as the original state, only the x and y
            coordinates are new.
        Tsc_initial = np.array([[1, 0, 0,   1  ],
                                [0, 1, 0,   1  ],
                                [0, 0, 1, 0.025],
                                [0, 0, 0,   1  ]])
        # Cube final configuration
        Tsc_goal = np.array([[ 0, 1, 0,   1  ],
                             [-1, 0, 0,  -1.0 ],
                             [ 0, 0, 1, 0.025],
                             [ 0, 0, 0,   1  ]])
                            
    Results: (Refers to video and Xerr graph)
        The video and graph is as expected the same as the best case, but now
        the block is just in a new pick and place location.
