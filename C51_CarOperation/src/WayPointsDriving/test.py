from Tkinter import *
import Tkinter
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import time
#bla = Tkinter.IntVar()




y3 = [0, 1, 0]
y4 = [0, 1, 1, 0]   
OrientationMF= [[-1,  -1,  -(60.0/180.0),  -(30.0/180.0)],[-(60.0/180.0),  -(30.0/180.0),  0],[-(5.0/180.0),  0,  (5.0/180.0)],[0,  30.0/180.0,  60.0/180.0],[30.0/180.0, 60.0/180.0,  1,  1]]#[VNO NO ZO PO VPO]   
#Distance
DistanceMF= [ [0, 0,  3.5, 7],  [3.5,  13.5, 23.5],   [20,  30,  35,  35]] #[CLOSE FAR VFAR] 

#Speed
SpeedMF = [ [0,  0,  0.3] ,  [0.2,  0.6,  1.0],   [0.6,  1,  1.4]] #[SLOW  FAST  VFAST ]


#Circular Speed
CircularSpeedMF = [ [-1.7,  -1.0,  -0.3],  [-0.75,  -0.3,  0],  [-0.1,  0,  0.1],  [0,  0.3,  0.75],   [0.3,   1.0,  1.7]] # [HL L  ST  R  HR]

class App:
    def __init__(self, master):
        self.flag=0
        self.top = Tkinter.Toplevel
        self.exitVal = 0
        self.DistVal =Tkinter.IntVar()
        # Create a container
        self.frame =master     #  Tkinter.Frame  
        #=============generate a menu bar================#,onvalue=True, offvalue=False
        menubar = Tkinter.Menu(self.frame)
        filemenu = Tkinter.Menu(menubar, tearoff=0)
        filemenu.add_separator()
        filemenu.add_command(label="Exit", command=self.exitGUI)
        menubar.add_cascade(label="File", menu=filemenu)
        
        Viewmenu = Tkinter.Menu(menubar, tearoff=0)

        self.DistBut = Viewmenu.add_checkbutton(label="Distance MF",  variable = self.DistVal)
        Viewmenu.add_checkbutton(label="Orientation MF")
        Viewmenu.add_checkbutton(label="Speed MF")
        Viewmenu.add_checkbutton(label="Steer MF", command=self.distancePlot)
        Viewmenu.add_separator()
        Viewmenu.add_checkbutton(label="View All", command=self.openAll)
        Viewmenu.add_command(label="Close All", command=self.closeAll)
        menubar.add_cascade(label="View", menu=Viewmenu)

        self.frame.config(menu=menubar)
        
        #===============menu generated================#
        upperframe=LabelFrame(self.frame, text="raw data")
        upperframe.pack(side = LEFT,expand=TRUE )
        #=========================================#
        #--------------------------------------Location---------------------------------------------#
        #=========================================#
        
        locationFrame = Frame(upperframe)
        locationFrame.pack(side = TOP,expand=TRUE )
        self.Location = Tkinter.StringVar()
        label1 = Tkinter.Message( locationFrame, text="Atlas's Current location :", relief=Tkinter.RAISED, width = 20000, fg="brown"  )
        label1.pack(side = LEFT)
        label2 = Tkinter.Message( locationFrame, textvariable=self.Location, relief=Tkinter.RAISED, width = 20000, fg="blue" )
        self.Location.set("Waitting to inititialize")
        label2.pack(side = LEFT)
        #=========================================#

        #=========================================#
        #-------------------------------------Actual Speed---------------------------------------------#
        #=========================================#
        
        locationFrame = Frame(upperframe)
        locationFrame.pack(side = TOP,expand=TRUE )
        self.ActSpeed = Tkinter.StringVar()
        labelsp = Tkinter.Message( locationFrame, text="Atlas's Actual Speed :", relief=Tkinter.RAISED, width = 20000, fg="brown"  )
        labelsp.pack(side = LEFT)
        labelsp2 = Tkinter.Message( locationFrame, textvariable=self.ActSpeed, relief=Tkinter.RAISED, width = 20000, fg="blue" )
        self.ActSpeed.set("Waitting to inititialize")
        labelsp2.pack(side = LEFT)
        #=========================================#
        
        #=========================================#
        #--------------------------------------Goal--------------------------------------------#
        #=========================================#
        
        objectFrame = Frame(upperframe)
        objectFrame.pack(side = TOP,expand=TRUE )
        self.NextObjective = Tkinter.StringVar()
        label0 = Tkinter.Message( objectFrame, text="Next Way Point Objective is :", relief=Tkinter.RAISED, width = 20000, fg="brown"  )
        label0.pack(side = LEFT)
        label = Tkinter.Message( objectFrame, textvariable=self.NextObjective, relief=Tkinter.RAISED, width = 20000, fg="blue" )
        self.NextObjective.set("Waitting to inititialize")
        label.pack(side = LEFT)
        #=========================================#
        
       

        
        #=========================================#
        #--------------------------------------Circular Speed FRAME---------------------------------------------#       
        #=========================================#
        
        CspeedFrame = Frame(upperframe)
        CspeedFrame.pack(side = BOTTOM,expand=TRUE )
        self.Cspeed = Tkinter.StringVar()
        label5 = Tkinter.Message( CspeedFrame, text="Hand Wheel Turn:", relief=Tkinter.RAISED, width = 20000, fg="brown"  )
        label5.pack(side = LEFT)
        label6 = Tkinter.Message( CspeedFrame, textvariable=self.Cspeed, relief=Tkinter.RAISED, width = 2000000, fg="blue" )
        self.Cspeed.set("Waitting to inititialize")
        label6.pack(side = LEFT)
        #=========================================#
        
        #=========================================#
        #--------------------------------------Speed FRAME---------------------------------------------#       
        #=========================================#
        
        speedFrame = Frame(upperframe)
        speedFrame.pack(side = BOTTOM,expand=TRUE )
        self.speed = Tkinter.StringVar()
        label3 = Tkinter.Message( speedFrame, text="Speed:", relief=Tkinter.RAISED, width = 20000, fg="brown"  )
        label3.pack(side = LEFT)
        label4 = Tkinter.Message( speedFrame, textvariable=self.speed, relief=Tkinter.RAISED, width = 2000000, fg="blue" )
        self.speed.set("Waitting to inititialize")
        label4.pack(side = LEFT)
        #=========================================#        
        #=========================================#
        #--------------------------------------Orientation FRAME---------------------------------------------#       
        #=========================================#
        
        dOriFrame = Frame(upperframe)
        dOriFrame.pack(side = BOTTOM,expand=TRUE )
        self.dOri = Tkinter.StringVar()
        do7 = Tkinter.Message( dOriFrame, text="dOrientation:", relief=Tkinter.RAISED, width = 20000, fg="brown"  )
        do7.pack(side = LEFT)
        do8 = Tkinter.Message( dOriFrame, textvariable=self.dOri, relief=Tkinter.RAISED, width = 2000000, fg="blue" )
        self.dOri.set("Waitting to inititialize")
        do8.pack(side = LEFT)
        #=========================================#
        #--------------------------------------Orientation FRAME---------------------------------------------#       
        #=========================================#
        
        OriFrame = Frame(upperframe)
        OriFrame.pack(side = BOTTOM,expand=TRUE )
        self.Ori = Tkinter.StringVar()
        label7 = Tkinter.Message( OriFrame, text="Orientation:", relief=Tkinter.RAISED, width = 20000, fg="brown"  )
        label7.pack(side = LEFT)
        label8 = Tkinter.Message( OriFrame, textvariable=self.Ori, relief=Tkinter.RAISED, width = 2000000, fg="blue" )
        self.Ori.set("Waitting to inititialize")
        label8.pack(side = LEFT)
        #=========================================#
        
        #=========================================#
        #--------------------------------------Distance FRAME---------------------------------------------#       
        #=========================================#
        
        DistanceFrame = Frame(upperframe)
        DistanceFrame.pack(side = BOTTOM,expand=TRUE )
        self.Distance = Tkinter.StringVar()
        label9 = Tkinter.Message( DistanceFrame, text="Distance:", relief=Tkinter.RAISED, width = 20000, fg="brown"  )
        label9.pack(side = LEFT)
        label10 = Tkinter.Message( DistanceFrame, textvariable=self.Distance, relief=Tkinter.RAISED, width = 2000000, fg="blue" )
        self.Distance.set("Waitting to inititialize")
        label10.pack(side = LEFT)
        #=========================================#    
        
        lowerFrame = LabelFrame(self.frame, text="All Way Points")
        lowerFrame.pack(side = BOTTOM,expand=TRUE )
        
        AllWPframe = Frame(lowerFrame)
        AllWPframe.pack(side = TOP,expand=TRUE )
        self.AllWP = Tkinter.StringVar()
        label11 = Tkinter.Message( AllWPframe, text="All Way Points in buffer:", relief=Tkinter.RAISED, width = 20000, fg="green"  )
        label11.pack(side = TOP)
        label12 = Tkinter.Label( AllWPframe, textvariable=self.AllWP, relief=Tkinter.RAISED,fg="blue",wraplength =306 )
        self.Distance.set("Waitting to inititialize")
        label12.pack(side = BOTTOM)        
        
    def exitGUI(self):
        self.exitVal = 1
        self.frame.destroy()
        #self.frame.pack()
    def updateLocation(self, location ):
        self.Location.set(str( location))
    def updateActSpeed(self, speed ):
        self.ActSpeed.set(str( speed))        
    def updateWP(self, WPlist ):
        self.AllWP.set(str( WPlist))
    def updateObject(self,object):
        self.NextObjective.set(str(object))
    def updateSpeed(self,speedval):
        speedval = float(int(speedval*100))/100
        self.speed.set(str(speedval))
    def updateCircularSp(self,Cspeedval):
        Cspeedval = float(int(Cspeedval*100))/100
        self.Cspeed.set(str(Cspeedval))
    def updateOrientation(self,orien):
        orien = int(orien)
        self.Ori.set(str(orien))
    def updatedO(self,orien):
        orien = int(orien)
        self.dOri.set(str(orien))        
    def updateDistance(self,val):
        val = int(val)
        self.Distance.set(str(val))            
    def closeAll(self):
        if self.DistVal.get():
           self.DistVal.set(0)
           self.distancePlot()
        
    def openAll(self):
        if not self.DistVal.get():
           self.DistVal.set(1)
           self.distancePlot()
        
    def distancePlot(self):
        
        if self.DistVal.get():
            print "hi"
            if not self.flag:
                self.top = Tkinter.Toplevel()
                #self.top.title("Distance MF: ")
                self.fig = Figure()
                self.Distanceax = self.fig.add_subplot(111)
                self.flag=1
                self.canvas = FigureCanvasTkAgg(self.fig,master=self.top)
                self.canvas.get_tk_widget().pack(side='bottom', fill='both', expand=3)
            #Distanceax.title("hi")
            if not self.top.winfo_exists():
                self.DistVal.set(0)
            for mem in DistanceMF:
                if len(mem)==3:
                    line, = self.Distanceax.plot(mem, y3)
                else:
                    line, = self.Distanceax.plot(mem, y4)
            
            self.canvas.draw()
            
        else:
            try:
                self.top.destroy()
                self.flag=0
            except:
                pass
    def dodo(self):
        print self.DistVal.get()

def task(root,app):
    app.distancePlot()
    try:
        app.canvas.draw()
    except:
        pass
        
    root.after(2,task, root, app)    
    
root = Tkinter.Tk()        
app = App(root)
root.after(20,task, root, app)    
root.mainloop()  
