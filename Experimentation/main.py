import PyLidar3
import time
import math
from scipy import stats
from tkinter import *
class PointData:
    def __init__(self,v1,v2,isCo=None):
        if isCo is None:
            self.x=round(math.cos(math.radians(v1))*v2)
            self.y=round(math.sin(math.radians(v1))*v2)
            self.dist=v2
            self.angle=v1
        else:
            self.x=int(v1)
            self.y=int(v2)
    def addRepere(self,v1,v2,set=None):
        if set is True:
            self.x+=int(v1)
            self.y+=int(v2)
        else:
            if v2==0:return self.x+v1
            else:return self.y+v1
    def distance(self,point2):
        return math.hypot(self.x-point2.x,self.y-point2.y)
    def moyenne(self,point2):
        self.x=round((self.x+point2.x)/2)
        self.y=round((self.y+point2.y)/2)
class CronoMili:
    def __init__(self):
        self.beforeTime=self.getTime()
        self.actuelTime=None
        self.deltaTime=0
    def topCrono(self):
        self.beforeTime=self.actuelTime
        self.actuelTime=self.getTime()
        self.deltaTime=self.actuelTime-self.beforeTime
        return self.deltaTime
    def crono(self):
        return self.actuelTime-self.getTime()
    def getTime(self):
        return int(round(time.time() * 1000))
class ObjectHuman:
    def __init__(self,object):
        self.objectActuel=self.objectBuffer=object
        self.traitement(object)
        self.crono=CronoMili()
    def is_equal(self,object):
        #if object.isHuman==False:return False #Testé avant
        if math.fabs((object.distance-self.objectActuel.distance)/object.distance)>0.3:return False#différence taille entre mesure <30%
        if math.hypot(self.objectActuel.pointMoyen.x-object.pointMoyen.x,self.objectActuel.pointMoyen.y-object.pointMoyen.y)>6*self.crono.crono():return False#vitesse entre mesure inferieur a 6mm/ms soit 6m/s soit 21.6km/h
        return True
    def traitement(self,objectAc):
        self.crono.topCrono()
        self.objectBuffer=self.objectActuel
        self.objectActuel=objectAc

class ObjectData:
    def __init__(self,points):
        self.points=points
        self.distance=None
        self.pointMoyen=None
        self.isHuman=None
    def traitement(self):
        self.distance=0.
        xSomme=0
        ySomme=0
        X=[]
        Y=[]
        for i in range(1,len(self.points)):
            self.distance+=self.points[i].distance(self.points[i-1])
            X.append(self.points[i].x)
            Y.append(self.points[i].y)
            xSomme+=self.points[i].x
            ySomme+=self.points[i].y
            self.pointMoyen = PointData(round(xSomme/len(self.points)),round(ySomme/len(self.points)),isCo=True)
        #self.slope, self.intercept, self.r_value, self.p_value, self.std_err = stats.linregress(X, Y)
        self.isHuman=(self.distance<800 and self.distance>200)
        return self
def traitement(data):
    listPoints=[]
    listObject=[]
    for i in range(360): listPoints.append(PointData(i,data[i]))
    bufferTest=True
    for i in range(360):
        #print(str(listPoints[i].distance(listPoints[i-1]))+"<"+str(listPoints[i].dist*0.0174524+20))
        if listPoints[i].distance(listPoints[i-1])<listPoints[i].dist*0.0174524+20:
            if bufferTest:
                bufferTest=False
                listObject.append(ObjectData([listPoints[i-1],listPoints[i]]))
            else:
                listObject[-1].points.append(listPoints[i])
        elif listPoints[i].distance(listPoints[i-2])<listPoints[i].dist*0.0174524+40:
            if bufferTest:
                bufferTest=False
                listObject.append(ObjectData([listPoints[i-2],listPoints[i]]))
            else:
                listObject[-1].points.append(listPoints[i])
        else: bufferTest=True
    for object in listObject:
        if len(object.points)>4:continue#object.traitement()
        else:listObject.remove(object)
    return listPoints,listObject



fenetre = Tk()
fenetre.title("Lidar")
canvas = Canvas(fenetre,
           width=1000,
           height=750)
canvas.pack(expand=YES, fill=BOTH)
Obj = PyLidar3.YdLidarX4("COM5") #PyLidar3.your_version_of_lidar(port,chunk_size)
rapport=2
if(Obj.Connect()):
    gen = Obj.StartScanning()
    t = time.time() # start time
    while (time.time() - t) < 60: #scan for 30 seconds
        canvas.delete("all")
        Rx=round(canvas.winfo_width()/2)
        Ry=round(canvas.winfo_height()/2)
        canvas.create_oval(Rx-10,Ry-10,Rx+10,Ry+10,fill="purple")
        data=next(gen)
        listPoint,listObject=traitement(data)
        ii=0
        for point in listPoint: canvas.create_rectangle(round(point.addRepere(Rx,0)/rapport),round(point.addRepere(Ry,1)/rapport),round(point.addRepere(Rx,0)/rapport),round(point.addRepere(Ry,1)/rapport),fill="red")
        for object in listObject:
            object.traitement()
            if object.isHuman:
                ii+=1
                canvas.create_text(round(object.pointMoyen.addRepere(Rx,0)/rapport), round(object.pointMoyen.addRepere(Ry,1)/rapport), text="Object possible Hummain "+str(ii), font="Arial 16 italic", fill="blue")
                for i in range(1,len(object.points)):
                    canvas.create_line(round(object.points[i].addRepere(Rx,0)/rapport),round(object.points[i].addRepere(Ry,1)/rapport),round(object.points[i-1].addRepere(Rx,0)/rapport),round(object.points[i-1].addRepere(Ry,1)/rapport), fill="green")

        fenetre.update_idletasks()
        fenetre.update()
        time.sleep(0.1)
    Obj.StopScanning()
    Obj.Disconnect()
else:
    print("Error connecting to device")
