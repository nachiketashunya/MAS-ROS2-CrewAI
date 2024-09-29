import logging

def ret_logger():
    # Create a logger
    logger = logging.getLogger('shared_logger')
    logger.setLevel(logging.INFO)  # Set the logging level

    # Create a file handler to write logs to a file
    log_handler = logging.FileHandler('/home/nachiketa/dup_auto_ass1/src/data/events.log', mode="w")
    log_handler.setLevel(logging.INFO)

    # Create a formatter and set it for the handler
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    log_handler.setFormatter(formatter)

    # Add the handler to the logger
    logger.addHandler(log_handler)

    return logger