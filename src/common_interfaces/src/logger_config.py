import logging
import threading
from queue import Queue
import atexit

class ThreadSafeLogger:
    def __init__(self, name, log_file_path):
        self.logger = logging.getLogger(name)
        self.logger.setLevel(logging.INFO)
        
        file_handler = logging.FileHandler(log_file_path)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(formatter)
        self.logger.addHandler(file_handler)
        
        self.queue = Queue()
        self.lock = threading.Lock()
        self.worker_thread = threading.Thread(target=self._worker)
        self.worker_thread.daemon = True
        self.worker_thread.start()
        
        atexit.register(self.shutdown)

    def _worker(self):
        while True:
            record = self.queue.get()
            if record is None:
                break
            self.logger.handle(record)

    def log(self, level, message):
        with self.lock:
            record = self.logger.makeRecord(
                self.logger.name, level, None, None, message, None, None
            )
            self.queue.put(record)

    def info(self, message):
        self.log(logging.INFO, message)

    def warning(self, message):
        self.log(logging.WARNING, message)

    def error(self, message):
        self.log(logging.ERROR, message)

    def shutdown(self):
        self.queue.put(None)
        self.worker_thread.join()

# Singleton instance
_logger_instance = None
_logger_lock = threading.Lock()

def get_logger(name="shared_logger", log_file_path="/home/nachiketa/dup_auto_ass1/src/data/events.log"):
    global _logger_instance
    with _logger_lock:
        if _logger_instance is None:
            _logger_instance = ThreadSafeLogger(name, log_file_path)
    return _logger_instance

# Example usage
if __name__ == "__main__":
    logger = get_logger()
    logger.info("This is a test log message")