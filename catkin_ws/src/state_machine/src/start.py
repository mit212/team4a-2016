class Start(State):
    def __init__(self, currentInput):
        self.currentInput = currentInput
        self.canStart = False
    
    def run(self):
        # wait for start signal - modify canStart
        self.canStart = True
    
    def nextInput(self):
        return 2

    def nextState(self):
        return Search

    def isFinished(self):
        return canStart

    def isStopState(self):
        return False
