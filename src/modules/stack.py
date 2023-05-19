from collections import deque

# Stack class
class Stack(deque):
    # Constructor
    def __init__(self):
        super().__init__()
    # Push
    def stackup(self, a):
        super().append(a)
    # Pop
    def unstack(self):
        return super().pop()
