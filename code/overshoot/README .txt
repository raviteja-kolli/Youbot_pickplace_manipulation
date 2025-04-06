## Capstone_Robotic_Manipulation

# Overshoot simulation

# Description:
This README is for the overshoot case

The python file `Milestone3.py` in the respective folders consistes of all 
three Milestones combined.

Cut and paste this command in the command line to generate the csv file:
(You must be in the same directory as the python file.)
    +  `python3 -m Milestone3.py`

1) overshoot:
    Type of controller: Feedforward with PI - Kp = 5.0, Ki = 1.0

    Results: (Refers to video and Xerr graph)
        The graph displays overshoot at the start of the simulation and at the
        pick (600 [ms]) and place (1200 [ms]) timestep. This is beacause of the
        controller gains not baing tuned correctly and creating to large control
        commands. The graph does not display a typical overshoot and sinosoidal
        decaying aoocilation as I could not determine the gains for this and ran
        out of time.
        
        Usually you will tune the Kp gain untill the system has a resonable
        response with minimal occilation and overshoot. The system will have a
        non zero steady state error. To decrease occilation frequency, decrease
        the pproportional gain and vice versa. Then add the integral gain to
        ensure azero steady state error and the derivative term to eliminate
        overshoot. The proportinal term can now be increase to decrease the 
        response time.
