
class AstarNode():

    def __init__(self, pos, index, parent):
        self.pos = pos
        self.index = index
        self.parent = parent
        self.g = 0.0
        self.h = 0.0
    
    def get_f(self):
        return self.g + self.h
    
    def __eq__(self, other):
        return self.pos == other.pos