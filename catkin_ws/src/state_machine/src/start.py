class Start(State):
    def __init__(self, currentInput):
        self.currentInput = currentInput
        self.nextInput = 2
        self.nextState = Search(nextInput)
        self.canStart = False
    
    def run(self):
        # wait for start signal - modify canStart
        pass
    
    def isFinished(self):
        return canStart
