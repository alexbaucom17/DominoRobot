import time

class NonBlockingTimer:

    def __init__(self, trigger_time):
        self.start_time = time.time()
        self.trigger_time = trigger_time

    def check(self):
        if time.time() - self.start_time > self.trigger_time:
            return True 
        else:
            return False

def write_file(filename, text):
    with open(filename, 'w+') as f:
        f.write(text)