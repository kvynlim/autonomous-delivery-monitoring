from datetime import datetime
from infrastructure.utility.singleton import Singleton
from pathlib import Path
from sys import stdout
import logging

class CustomFormatter(logging.Formatter):
    green = "\x1b[1;32m"
    grey = "\x1b[38;20m"
    yellow = "\x1b[33;20m"
    red = "\x1b[31;20m"
    bold_red = "\x1b[31;1m"
    reset = "\x1b[0m"
    format = "[%(asctime)s] [%(levelname)s] %(message)s"

    FORMATS = {
        logging.DEBUG: grey + format + reset,
        logging.INFO: green + format + reset,
        logging.WARNING: yellow + format + reset,
        logging.ERROR: red + format + reset,
        logging.CRITICAL: bold_red + format + reset
    }

    def format(self, record):
        return logging.Formatter(self.FORMATS.get(record.levelno)).format(record)

class Logger(metaclass = Singleton):
    def __init__(self):
        self.__logger = logging.getLogger(__name__)
        self.__logger.setLevel("DEBUG")
        Path("logs").mkdir(parents = True, exist_ok = True)

        now = datetime.now()
        name = str(now.year) + str(now.month).zfill(2) + str(now.day).zfill(2) + str(now.hour).zfill(2) + str(now.minute).zfill(2) + str(now.second).zfill(2)
        fh = logging.FileHandler(f"logs/{name}.txt")
        sh = logging.StreamHandler(stdout)
        fh.setFormatter(logging.Formatter("[%(asctime)s] [%(levelname)s] %(message)s"))
        sh.setFormatter(CustomFormatter())
        fh.setLevel("DEBUG")
        sh.setLevel("DEBUG")
        self.__logger.addHandler(fh)
        self.__logger.addHandler(sh)

    def critical(self, string: str):
        self.__logger.critical(string)

    def debug(self, string: str):
        self.__logger.debug(string)
    
    def error(self, string: str):
        self.__logger.error(string)

    def info(self, string: str):
        self.__logger.info(string)

    def warning(self, string: str):
        self.__logger.warning(string)