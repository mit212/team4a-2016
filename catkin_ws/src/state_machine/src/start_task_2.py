from state import State
from search import Search
from drive_distance import DriveDistance
from prepare_catch import PrepareToCatch
from turn_tag_position import TurnTagPosition

class Start(State):
    def __init__(self, current_input):
        self.current_input = current_input
        self.can_start = False
    
    def run(self):
        self.can_start = True
    
    def next_input(self):
        return 9

    def next_state(self):
        return Search(self.next_input())

    def is_finished(self):
        return self.can_start

    def is_stop_state(self):
        return False

    def __str__(self):
        return "Start(%s)" % (self.current_input)
