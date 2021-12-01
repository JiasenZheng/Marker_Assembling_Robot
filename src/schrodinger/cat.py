"""This class acts as a Qboolean in which the nature of an object is undetermined and needs to be checked."""


class cat:
    def __init__(self, observed = False, Nature = False):
        self.qbool = [observed, Nature]
    def __str__(self):
        if self.qbool[0] == False:
            return "Undetermined"
        else:
            return f"{self.qbool[1]}"
    def getBox(self, i):
        """Gets an attribute of the cat."""
        return self.qbool[i]

    def openBox(self, status):
        """Specifies that the status of the cat has been determined."""
        self.qbool = [True, status]

    def closeBox(self):
        """Makes the status of the cat unknonw for the instance of reseting confidence."""
        self.qbool[0] = False
