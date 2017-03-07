import math
class Vertex(object):
    
    def __init__(self, nodeId, nodeName, x, y):
        """ initializes a vertex object """
        self.nodeId = nodeId
        self.nodeName = nodeName
        self.x = x
        self.y = y
        self.dirc = {}
        
    def __str__(self):
        res = "["+str(self.nodeName)+"]: "+str(self.nodeId) + "(" + str(self.x) + "," + str(self.y)+")"
        return res
    def setitem(self,key,value):
        self.dirc[key]=value
    def getitem(self,key):
        return self.dirc[key]
    
def distance(a,b):
    lx = abs(a.x - b.x)
    ly = abs(a.y - b.y)
    lz = int(math.sqrt( math.pow(lx,2)+math.pow(ly,2)))
    return lz

def angle(a,b):
    lx = abs(a.x - b.x)
    ly = abs(a.y - b.y)
    if lx == 0:
        if a.y > b.y:  #y轴正向
            return 180
        else:           #y轴负向
            return 0
    else:
        radian = math.atan(ly/lx)
        angle = int(math.degrees(radian))
        if b.x > a.x and a.y == b.y : #x轴正向
            return 90
        elif b.x<a.x and a.y==b.y:  #x轴负向
            return 270
        elif a.x<b.x and a.y<b.y: #第一象限
            return 90-angle
        elif a.x > b.x and b.y>a.y : #第四象限
            return 270+angle
        elif a.x > b.x and a.y >b.y : #第三象限
            return 270-angle
        elif b.x > a.x and a.y > b.y : #第二象限
            return 90+angle

#v1 = Vertex('1',"A",0,0)
#v1.setitem("B",1)
#v2 = Vertex('2',"B",-5,-5)
#print(distance(v1,v2))
#print(angle(v1,v2))
#print(v1.getitem("B"))



#Sample test of basic functionalities given
#v1 = Vertex("A", 100, 200)
#print(v1)
