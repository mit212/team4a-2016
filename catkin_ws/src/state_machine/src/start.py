from state import State
from search import Search

class Start(State):
    def __init__(self, current_input):
        self.current_input = current_input
        self.can_start = False
    
    def run(self):
        # wait for start signal - modify canStart
        self.can_start = True
    
    def next_input(self):
        return 2

    def next_state(self):
        return Search(self.next_input())

    def is_finished(self):
        return self.can_start

    def is_stop_state(self):
        return False