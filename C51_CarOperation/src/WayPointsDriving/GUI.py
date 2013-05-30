from Tkinter import *
import Tkinter
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

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
        self.MAPflag=0
        self.top = Tkinter.Toplevel
        self.exitVal = 0
        self.DistVal =Tkinter.IntVar()
        # Create a container
        self.frame =master     #  Tkinter.Frame  
        self.frame.title("Driving GUI: ")

        #=============generate a menu bar================#,onvalue=True, offvalue=False
        menubar = Tkinter.Menu(self.frame)
        filemenu = Tkinter.Menu(menubar, tearoff=0)
        filemenu.add_separator()
        filemenu.add_command(label="Exit", command=self.exitGUI)
        menubar.add_cascade(label="File", menu=filemenu)
        
        
        MAPmenu = Tkinter.Menu(menubar, tearoff=0)
        self.MAPVal =Tkinter.IntVar()

        MAPmenu.add_checkbutton(label="ShowMap",  variable = self.MAPVal)
        
        menubar.add_cascade(label="MAP", menu=MAPmenu)

        
        
        Viewmenu = Tkinter.Menu(menubar, tearoff=0)

        self.DistBut = Viewmenu.add_checkbutton(label="Distance MF",  variable = self.DistVal)
        Viewmenu.add_checkbutton(label="Orientation MF")
        Viewmenu.add_checkbutton(label="Speed MF")
        Viewmenu.add_checkbutton(label="Steer MF")
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
    def showMAP(self, points):
        if self.MAPVal.get():
                #print "hi"
                if not self.MAPflag:
                    self.top = Tkinter.Toplevel()
                    self.top.title("Obstical MAP ")
                    self.fig = Figure()

                    
                    self.Distanceax = self.fig.add_subplot(111)
                    
                    #self.Distanceax.xaxis([-15, 15])
                    
                    #print dir(self.Distanceax)
                    self.MAPflag=1
                    self.canvas = FigureCanvasTkAgg(self.fig,master=self.top)
                    self.canvas.get_tk_widget().pack(side='bottom', fill='both', expand=3)
                #Distanceax.title("hi")
                if not self.top.winfo_exists():
                    self.DistVal.set(0)
    #            for pt in points:
    #                if len(mem)==3:
    #                    line, = self.Distanceax.plot(mem, y3)
    #                else:
    #                    line, = self.Distanceax.plot(mem, y4)
                x2 = []
                y2 = []
                #print dir(self.Distanceax)
                #print self.Distanceax.ishold()
                self.Distanceax.hold(False)
                for pt in points:
                    #if (pt[1]>-0.2 and pt[1]<1.8) and pt[0]<6:
                        x2.append(pt[0])
                        y2.append(pt[1])
                line,  = self.Distanceax.plot(y2, x2, "bo")
                self.Distanceax.set_xlim([-5, 5])
                self.Distanceax.set_ylim([0, 15])
                self.Distanceax.set_xticks(ticks=[ -5, -4, -3, -2, -1,0, 1, 2, 3, 4,  5])
                self.Distanceax.set_yticks(ticks=[0, 2, 4, 6, 8, 10, 15 ])
                self.Distanceax.set_xlabel("Left         Right")
                self.Distanceax.set_ylabel("Astern           Ahead")
                #print dir(self.Distanceax.plot) 
                self.canvas.draw()

        else:
            try:
                self.top.destroy()
                self.MAPflag=0
            except:
                pass
    def closeAll(self):
        if self.DistVal.get():
           self.DistVal.set(0)
           self.distancePlot()
    
    def openAll(self):
        if not self.DistVal.get():
           self.DistVal.set(1)
           self.distancePlot()
        
    def distancePlot(self, points, WP, wp):
        if self.DistVal.get():
            #print "hi"
            if not self.MAPflag:
                self.top = Tkinter.Toplevel()
                self.top.title("Obstical MAP ")
                self.fig = Figure()
                print dir(self.fig)
                self.fig.set_label("Left         Right")
                self.Distanceax = self.fig.add_subplot(111)
                #self.Distanceax.xaxis([-15, 15])
                
                #print dir(self.Distanceax)
                self.MAPflag=1
                self.canvas = FigureCanvasTkAgg(self.fig,master=self.top)
                self.canvas.get_tk_widget().pack(side='bottom', fill='both', expand=3)
            #Distanceax.title("hi")
            if not self.top.winfo_exists():
                self.DistVal.set(0)
#            for pt in points:
#                if len(mem)==3:
#                    line, = self.Distanceax.plot(mem, y3)
#                else:
#                    line, = self.Distanceax.plot(mem, y4)
            x2 = []
            y2 = []
            #print dir(self.Distanceax)
            self.Distanceax.clear()
            for pt in points:
                #if (pt[1]>-0.2 and pt[1]<1.8) and pt[0]<6:
                    x2.append(pt[0])
                    y2.append(pt[1])
            line,  = self.Distanceax.plot(y2, x2, "bo")
            #print dir(self.Distanceax.plot) 
            self.canvas.draw()
        else:
            try:
                self.top.destroy()
                self.MAPflag=0
            except:
                pass
#        if self.DistVal.get():
#            print "hi"
#            self.top = Tkinter.Toplevel()
#            #self.top.title("Distance MF: ")
#            fig = Figure()
#            Distanceax = fig.add_subplot(111)
#            #Distanceax.title("hi")
#            for mem in DistanceMF:
#                if len(mem)==3:
#                    line, = Distanceax.plot(mem, y3)
#                else:
#                    line, = Distanceax.plot(mem, y4)
#            canvas = FigureCanvasTkAgg(fig,master=self.top)
#            canvas.show()
#            canvas.get_tk_widget().pack(side='bottom', fill='both', expand=3)
#        else:
#            try:
#                self.top.destroy()
#            except:
#                pass
    def dodo(self):
        print self.DistVal.get()
        
        '''''
        

        # Create 2 buttons
        self.button_left = Tkinter.Button(self.frame,text="< Decrease Slope",
                                        command=self.decrease)
        self.button_left.pack(side="left")
        self.button_right = Tkinter.Button(self.frame,text="Increase Slope >",
                                        command=self.increase)
        self.button_right.pack(side="left")
        
        
        self.L = Tkinter.Label(self.frame, text="X = 0")
        self.L.pack( side = "left")   
        self.R = Tkinter.Label(self.frame, text="Y = 0")
        self.R.pack( side = "left")   
       
        fig = Figure()
        Orientationax = fig.add_subplot(221)
        for mem in OrientationMF:
            if len(mem)==3:
                self.line, = Orientationax.plot(mem, y3)
            else:
                self.line, = Orientationax.plot(mem, y4)
        
        Distanceax = fig.add_subplot(222)
        #Distanceax.title("hi")
        for mem in DistanceMF:
            if len(mem)==3:
                self.line, = Distanceax.plot(mem, y3)
            else:
                self.line, = Distanceax.plot(mem, y4)
        
        CircularSpeedax = fig.add_subplot(223)
        for mem in CircularSpeedMF:
            if len(mem)==3:
                self.line, = CircularSpeedax.plot(mem, y3)
            else:
                self.line, = CircularSpeedax.plot(mem, y4)
                
        speedax = fig.add_subplot(224)
        for mem in SpeedMF:
            if len(mem)==3:
                self.line, = speedax.plot(mem, y3)
            else:
                self.line, = speedax.plot(mem, y4)
        
        self.canvas = FigureCanvasTkAgg(fig,master=master)
        self.canvas.show()
        self.canvas.get_tk_widget().pack(side='bottom', fill='both', expand=3)
        
        '''
