from datetime import datetime
import time
FORMAT = '%H:%M:%S.%f'
class LOG:
    def __init__(self):
        self.WayPointLst=[]
        self.DistanceErrorLst=[]
        self.MyPathLst=[]
        self.PassedWayPointLst=[]

    def WayPoint(self, obj):
        self.WayPointLst.append([obj[0], obj[1]])#, datetime.now().strftime(FORMAT)
    def PassedWayPoint(self, obj):
        self.PassedWayPointLst.append([obj[0], obj[1], datetime.now().strftime(FORMAT)])
    def MyPath(self, x, y):
        self.MyPathLst.append([x, y, datetime.now().strftime(FORMAT)])
    def DistanceError(self, dist):
        self.DistanceErrorLst.append([dist, datetime.now().strftime(FORMAT)])
