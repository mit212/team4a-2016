from abc import ABCMeta, abstractmethod

class State:
    __metaclass__ = ABCMeta

    @abstractmethod
    def run(self):
        pass

    @abstractmethod
    def isFinished(self):
        return False

class Start(State):
    def __init__(self, currentInput):
        self.currentInput = currentInput
        self.nextInput = 3
        self.nextState = Middle
        self.stopState = False

    def run(self):
        self.currentInput += 1
        print "start", self.currentInput

    def isFinished(self):
        return self.currentInput == 3

class Middle(State):
    def __init__(self, currentInput):
        self.currentInput = currentInput
        self.nextState = End
        self.stopState = False
        if currentInput == 3:
            self.nextInput = 5
        else:
            self.nextInput = 1

    def run(self):
        self.currentInput -= 1
        print "middle", self.currentInput

    def isFinished(self):
        return self.currentInput == 0

class End(State):
    def __init__(self, currentInput):
        self.currentInput = currentInput
        self.stopState = True
        self.nextState = End
        self.nextInput = 0

    def run(self):
        self.currentInput += 1
        print "stop", self.currentInput

    def isFinished(self):
        return self.currentInput == 5


currentState = Start(0)
while not currentState.stopState:
    while not currentState.isFinished():
        currentState.run()
    print "next input:", currentState.nextInput
    currentState = currentState.nextState(currentState.nextInput)