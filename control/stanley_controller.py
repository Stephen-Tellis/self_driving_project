"""
Inspired by: (https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)
           : Atsushi Sakai (@Atsushi_twi)
"""

import math

from logger.logger_config import setup_logger

# Setup Logging
logger = setup_logger(__name__)

class SteeringControl:
    
    def __init__(self, state, trajectory) -> None:
        self.state = state
        self.trajectory = trajectory