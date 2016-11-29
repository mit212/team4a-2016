from state import State

class Stop(State):
    def __init__(self, current_input):
        self.current_input = current_input
    
    def run(self):
        pass
    
    def next_input(self):
        return 0

    def next_state(self):
        return None

    def is_finished(self):
        return True

    def is_stop_state(self):
        return True

    def __str__(self):
        return "Stop(%s)" % (self.current_input)
