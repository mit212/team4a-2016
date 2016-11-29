from state import State
from search import Search
from catch import Catch

class Start(State):
    def __init__(self, current_input):
        self.current_input = current_input
        self.can_start = False
    
    def run(self):
        # wait for start signal - modify canStart
        self.can_start = True
    
    def next_input(self):
        return 2.5 # for actual thing
        #return 6 # for testing drive

    def next_state(self):
        return Catch(self.next_input())

    def is_finished(self):
        return self.can_start

    def is_stop_state(self):
        return False

    def __str__(self):
        return "Start(%s)" % (self.current_input)
