import numpy as np
"""
Provides a class to generate and map X, Y and Z coordinates to locations on 
and above a defined tray.
"""
class TrayPositions:
    def __init__(self, centerX, centerY, pitchX, pitchY, zList, rpy=None, shape=(3,3)):
        """
        Instantiating tray locations' x, y and z coordinates from tray 
        dimensions.
        Sets the roll pictch yaw required of the end effector when picking 
        from the tray.
        """
        self.centerX = centerX
        self.centerY = centerY
        self.pitchX = pitchX
        self.pitchY = pitchY
        self.shape = shape + (2,)
        self.locsXY = self.fill_out_moves()
        self.locsZ = zList
        self.rpy = rpy
        
    def fill_out_moves(self):
        """
        Calculates and returns an array of X,Y coordinates for each location 
        in the tray.
        Return is a 1D array of X,Y coordinates for each location.
        """

        ## Identifying the center element in locations array given the shape
        centerLocX = np.median(np.arange(self.shape[1]))
        centerLocY = np.median(np.arange(self.shape[0]))

        ## Setting up to create array of locations with x, y coords at/for each locations
        locations = np.zeros(self.shape)
        for row in range(self.shape[0]):
            for col in range(self.shape[1]):
                locX = (row - centerLocX) * self.pitchX + self.centerX
                locY = (col - centerLocY) * self.pitchY + self.centerY
                locations[row, col] = np.array([locX, locY])
        
        ## "flattened" into array of coords
        locs = locations.reshape((self.shape[0]*self.shape[1], 2))
        return locs

    def get_location(self, location_index, zlevel):
        """ 
        Fetches x, y, z coordinates given a O-indexed tray location and a 
        0-indexed height level.
        """
        print(self.locsXY[location_index])
        x, y = tuple(self.locsXY[location_index])
        z = self.locsZ[zlevel]
        ## NEED TO CHECK FOR AND PREVENT INDEX ERRORS
        return x, y, z

    def get_tray_size(self):
        """ Returns the size of the 1D tray"""
        return self.locsXY.shape[0]

    def get_zLevels(self):
        """ Returns the list of height levels instantiated for a tray"""
        return self.locsZ
    
    def get_zLevelCount(self):
        """ Returns the number of height levels instantiated for the tray"""
        return len(self.locsZ)

    def get_rpy(self):
        """ Returns the roll pitch yaw instantiated for the tray"""
        return self.rpy

if __name__ == "__main__":
    """
    Testing the generation of tray location coordinates.
    """
    tp = TrayPositions(0.5, 0.5, 0.25, 0.25, [0.5, 0.3])
    print(tp.locsXY)
    # for i in range(tp.get_tray_size()):
    #     for j in range(tp.get_zLevelCount()):
    #         print(i,j, tp.get_location(i,j))