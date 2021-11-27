import numpy as np

class TrayPositions:
    def __init__(self, centerX, centerY, pitchX, pitchY, standoffZ, pickplaceZ, shape=(3,3)):
        self.centerX = centerX
        self.centerY = centerY
        self.pitchX = pitchX
        self.pitchY = pitchY
        self.shape = shape + (2,)
        self.locsXY = self.fill_out_moves()
        self.locsZ = [pickplaceZ, standoffZ]
        
    def fill_out_moves(self):
        centerLocX = np.median(np.arange(self.shape[1]))
        centerLocY = np.median(np.arange(self.shape[0]))
        locations = np.zeros(self.shape)
        for row in range(self.shape[0]):
            for col in range(self.shape[1]):
                locX = (row - centerLocX) * self.pitchX + self.centerX
                locY = (col - centerLocY) * self.pitchY + self.centerY
                locations[row, col] = np.array([locX, locY])
        locs = locations.reshape((self.shape[0]*self.shape[1], 2))
        return locs

    def get_location(self, location_index, zlevel=1):
        """ locations are O-indexed """
        print(self.locsXY[location_index])
        x, y = tuple(self.locsXY[location_index])
        z = self.locsZ[zlevel]
        return x, y, z

    def get_tray_size(self):
        return self.locsXY.shape[0]

    def get_zLevels(self):
        return self.locsZ

if __name__ == "__main__":
    tp = TrayPositions(0.5, 0.5, 0.25, 0.25, 0.5, 0.3)
    for i in range(tp.get_tray_size()):
        for j in range(len(tp.get_zLevels())):
            print(i,j, tp.get_location(i,j))