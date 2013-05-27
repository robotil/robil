import datetime
import time

FORMAT = '%H:%M:%S.%f'
class LOG:
    def __init__(self):
        self.WayPointLst=[]
        self.DistanceErrorLst=[]
        self.MyPathLst=[]
        self.PassedWayPointLst=[]
        self.StartDrive=datetime.datetime.now().time()
        self.FinishDrive=datetime.datetime.now().time()
    
    def WayPoint(self, obj):
        self.WayPointLst.append([obj[0], obj[1]])
    def PassedWayPoint(self, obj):
        self.PassedWayPointLst.append([obj[0], obj[1], datetime.datetime.now().time()])
    def MyPath(self, x, y):
        self.MyPathLst.append([x, y, datetime.datetime.now().time()])
    def DistanceError(self, dist):
        dist = round(dist, 2)
        self.DistanceErrorLst.append([dist, datetime.datetime.now().time()])
    def getTime(self):
        tm=datetime.datetime.now().time()
        return tm
    def Difference(self, tm, tm2):
        fulldate1 = datetime.datetime(100, 1, 1, tm.hour, tm.minute, tm.second)
        fulldate2 = datetime.datetime(100, 1, 1, tm2.hour, tm2.minute, tm2.second)
        diff =  (fulldate2 - fulldate1)
        return diff
    def print2file(self):
        print "Generating file"
        f= open("log.txt", "w")
        f.write("Started Driving at:")
        f.write(str(self.StartDrive))
        f.write('\n')
        f.write("Finished Driving at:")
        f.write(str(self.FinishDrive))
        f.write('\n')
        f.write("Drove for in seconds:")
        diff = self.Difference(self.StartDrive, self.FinishDrive)
        f.write(str(diff.total_seconds()))
        f.write('\n')

        f.write("Actually drove through the following way points:\n")
        i=1
        for obj in self.PassedWayPointLst:
            f.write("%2d) "%i)
            i+=1
            f.write(str(obj[0]))
            f.write(',')
            f.write(str(obj[1]))
            f.write(':  ')
            f.write(str(obj[2]))
            f.write('\n')       
        f.write('\n')       
        f.write("Distances errors are:\n")
        i=1
        for obj in self.DistanceErrorLst:
            f.write("%2d) "%i)
            i+=1            
            f.write(str(obj[0]))
            f.write(':  ')
            f.write(str(obj[1]))
            f.write('\n')       
        f.write('\n')
        f.write("List of all of the way points given:\n")
        f.write(str(self.WayPointLst))
        f.write('\n\n')
        
        f.close()
