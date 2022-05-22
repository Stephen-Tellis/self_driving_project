"""! @brief One interface to the self driving car project"""
##
# @section description_main Description
# This file is one settings/run interface to every planner available
#
##
# @file main.py
#
# @section libraries_main Libraries/Modules
# - json
# - numpy
#
# @section author_main Author(s)
# - Created by Stephen Tellis.
#

# Local and Self made classes
from render_engine.env_renderer import RenderEnvironment, Resources
from path_planning import cubic_spline_planner
from vehicle_models.bicycle_model import AbstractCar
from logger.logger_config import setup_logger

# Python Libs
import json

# Installed Libs

# All settings go here (Controller gains are in their respective classes)
CONTROLLER = "pure pursuit"
TEACH = False
SHOW_TRUE_PATH = False  # Not relevant in teach mode, for debug
FPS = 20
DT = 1/FPS  # time tick
WB = 26  # [px] wheel base of vehicle
V_MAX = 50  # [px/s]
CAR_MODEL = Resources.GREEN_CAR  # Available are red and green
PATH_FILE = "path_planning/saved_path.txt"

# Setup Logging
logger = setup_logger(__name__)

# Custom imports based on settings
if CONTROLLER == "pure pursuit":
    from control.proportional_control import LongitudionalControl
    from control.pure_pursuit_control import SteeringControl
elif CONTROLLER == "stanley":
    from control.proportional_control import LongitudionalControl
    from control.stanley_controller import SteeringControl

with open(PATH_FILE) as f:
    """
    Get path seed points
    """
    PATH = json.load(f)


class PlayerCar(AbstractCar):
    IMG = CAR_MODEL
    START_POS_X = 180
    START_POS_Y = 200


def main():

    # Create a true path from seed points
    true_path = []
    if not TEACH:
        ax, ay = zip(*PATH)
        cx_t, cy_t, cyaw, _, _ = cubic_spline_planner.calc_spline_course(
            ax, ay, ds=2)
        cx = [int(i) for i in cx_t]
        cy = [int(i) for i in cy_t]
        lastIndex = len(cx) - 1
        for pos in zip(cx, cy):
            true_path.append(pos)

    # Add assets and their locations in the simulated environment go here
    env_images = [(Resources.GRASS, (0, 0)), (Resources.TRACK, (0, 0)),
                  (Resources.FINISH, (130, 250)), (Resources.TRACK_BORDER, (0, 0))]

    # Instantiate render engine, vehicle and controllers
    window = RenderEnvironment(
        CONTROLLER, env_images, CAR_MODEL, TEACH,
        SHOW_TRUE_PATH, true_path)
    if not TEACH:
        state = PlayerCar(dt=DT, wb=WB)
        p_control = LongitudionalControl()
        steering_control = SteeringControl(cx, cy, WB, cyaw)
        target_ind, _ = steering_control.search_target_index(
            state.rear_x, state.rear_y, state.yaw, state.v)

    # Initialize start time
    clock = window.setup_clock()

    run = True

    # The display and car control loop
    while run:

        clock.tick(FPS)

        # calculate control input
        if TEACH:
            run = window.update_display()
        else:
            if not lastIndex > target_ind:
                run = False
                logger.info("End of planned path reached")
                break
            ai = p_control.control(V_MAX, state.v)
            di, target_ind = steering_control.control(
                target_ind, state.rear_x, state.rear_y,
                state.yaw, state.v, state.wb)
            state.update(ai, di)  # Control car

            # update
            text_string = f"Current Speed: {state.v:.2f} px/s, Steering Angle: {di:.3f} rad"
            run = window.update_display(
                text_string, state.x, state.y, state.yaw)

    path_taught = window.quit_render()
    if TEACH:
        with open(PATH_FILE, "w") as f:
            json.dump(path_taught, f)
            logger.debug(path_taught)


if __name__ == "__main__":

    main()
