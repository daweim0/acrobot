import numpy as np
import random


class State:
    """
    Stores an acrobot state with torque and meta-information like time and id-number. The torque stored is
    the torque that was actively applied when the acrobot was in the stored state, not the torque that
    lead the the acrobot being in the stored state.
    """
    def __init__(self, angles: ()=np.zeros([5], dtype=np.float64), time: float=0, id_num: int=None, gyrodata=None):
        self.angles = angles
        self.time = time
        self.id_num = id_num
        self.gyrodata = gyrodata

        if self.id_num is None:
            id_num = random.randint(-2**30, 2**30)

    def __getitem__(self, item):
        return self.angles[item]

    def __setitem__(self, key, value):
        if key < 0 or key > 4:
            raise IndexError
        else:
            self.angles[key] = value

    def get_angles(self):
        return self.angles

    def __str__(self):
        return "State: " + str(self.angles) + ", time= " + str(self.time) + " id num= " + str(self.id_num) \
               + " gyro data= " + str(self.gyrodata)

    def __len__(self):
        return len(self.angles)
