import numpy as np

class TrayPositions:
    # def __init__(self, centerX, centerY, pitchX, pitchY, standoffZ, pickplaceZ, shape=(3,3)):
    def __init__(self, centerX, centerY, pitchX, pitchY, zList, rpy=None, shape=(3,3)):
        """
        Instantiating trayPosition Location array (self.locsXY) from tray dimensions
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
        """ locations are O-indexed, variable count of heights"""
        print(self.locsXY[location_index])
        x, y = tuple(self.locsXY[location_index])
        z = self.locsZ[zlevel]
        ## NEED TO CHECK FOR AND PREVENT INDEX ERRORS
        return x, y, z

    def get_tray_size(self):
        return self.locsXY.shape[0]

    def get_zLevels(self):
        return self.locsZ
    
    def get_zLevelCount(self):
        return len(self.locsZ)

    def get_rpy(self):
        return self.rpy

if __name__ == "__main__":
    tp = TrayPositions(0.5, 0.5, 0.25, 0.25, [0.5, 0.3])
    print(tp.locsXY)
    # for i in range(tp.get_tray_size()):
    #     for j in range(tp.get_zLevelCount()):
    #         print(i,j, tp.get_location(i,j))