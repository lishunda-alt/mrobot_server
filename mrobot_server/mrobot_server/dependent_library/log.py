"""
操作日志记录
"""
import time
from loguru import logger
from pathlib import Path


class get_logger:
    project_path = Path.cwd()
    log_path = Path(project_path, "log")
    __instance = None

    logger.add(f"{log_path}/AGV_logs.log", rotation="10 MB", encoding="utf-8", enqueue=True,
               retention="3 days", level="INFO")

    def __new__(cls, *args, **kwargs):
        if not cls.__instance:
            cls.__instance = super(get_logger, cls).__new__(cls, *args, **kwargs)

        return cls.__instance

    def info(self, msg):
        return logger.info(msg)

    def debug(self, msg):
        return logger.debug(msg)

    def warning(self, msg):
        return logger.warning(msg)

    def error(self, msg):
        return logger.error(msg)


log = get_logger()