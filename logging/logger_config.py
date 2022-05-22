"""! @brief A utility that sets up logging for us."""
##
# @section description_logger_config Description
# This file sets up logger formats, level etc
# Change level/format/add logfile here to reflect accross all classes in your package
#
##
# @file logger_config.py
#
# @section libraries_logger_config Libraries/Modules
# - logging
#
# @section author_logger_config Author(s)
# - Created by Stephen Tellis.
#

import logging

# Setup Logging


def setup_logger(name, filename=None):
    """!
    Returns a logger object for the file.
    @param name Pass the __name__ of the file you desire to setup logging on
    @param filename Full filepath to the log file
    
    @return logger object
    """
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)
    formatter = logging.Formatter(
        """[%(levelname)s] [%(asctime)s] [%(filename)s]: %(message)s""")
    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(formatter)
    if isinstance(filename, str):
        file_handler = logging.FileHandler(filename)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
    logger.addHandler(stream_handler)
    return logger
