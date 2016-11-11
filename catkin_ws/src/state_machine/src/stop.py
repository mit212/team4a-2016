from state import State

class Stop(State):
    def __init__(self, current_input):
        self.current_input = current_input
        self.can_start = False
    
    def run(self):
        self.can_start = True
    
    def next_input(self):
        return 0

    def next_state(self):
        return None

    def is_finished(self):
        return self.can_start

    def is_stop_state(self):
        return True
