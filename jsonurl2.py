import urllib.request
import json
import math
import GraphTest
import vertexTest
import serial
import time

#from espeak import espeak

#define hong
ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)
hello=0x12
ack=0x11
headingCtrl = 0x21
longl=0x23
longr=0x24
left=0x42
right=0x43
straight=0x44
stop=0x41
zero=0x26
ser.write(hello.to_bytes(1, byteorder='big'))
response=ser.readline()
while int.from_bytes(response, byteorder='big')!=0x11:
    ser.write(hello.to_bytes(1, byteorder='big'))
    response=ser.readline()
ser.write(ack.to_bytes(1, byteorder='big'))
print("received from arduino: ack")
print("send to arduino: ack")

string = ''

#espeak.synth('please input the building you want: ')
building = input('please input the building you want: ')
#espeak.synth('please input the level you want: ')
level = input('please input the level you want: ')
url= 'http://showmyway.comp.nus.edu.sg/getMapInfo.php?Building='+building+'&Level='+level
response = urllib.request.urlopen(url)
BuildingInfo = response.read().decode('UTF-8')
jsondata = json.loads(BuildingInfo)
node = jsondata["map"]
orientation=jsondata["info"]['northAt']
string = 'the number of nodes is '+str(len(node))
#espeak.synth(string)
print(string)
string = "the first node:" + node[0]["nodeName"]
#espeak.synth(string)
print(string)

G1 = GraphTest.Graph()

for i in node:
    nodeId = str(i['nodeId'])
    nodeName = str(i['nodeName'])
    X = int(i['x'])
    Y = int(i['y'])
    G1.addvertex(nodeId, nodeName, X, Y)

for i in node:
    name = str(i['nodeId'])
    Next = str(i['linkTo']).replace(' ','').split(',')
    for a in range(len(Next)):
        G1.addedge(name, Next[a])

"""
#Print all of the nodes that have been saved in Graph G1 Object
for realNodeId in G1.binary:
    print(G1.binary[realNodeId])
    print(G1.binary[realNodeId].dirc)
"""

def rotationAngle(v1,v2,orien,heading):
    orien=int(orien)
    angle=vertexTest.angle(v1,v2)
    varTemp=(heading+orien)%360
    if varTemp>180 and angle==0:
        angle=360
    elif angle-varTemp<-180:
        varTemp=varTemp-360
    elif varTemp==0 and angle>180:
        varTemp=360
    elif angle-varTemp>180:
        varTemp=varTemp+360
    realAngle=angle-varTemp
    return realAngle

#Code section:Judge turn left or turn right or go straight
def rotationDirec(nextangle):
    if nextangle>0:
        rotationDirec=" please turn right " + str(nextangle) + " degrees"
    elif nextangle<0:
        rotationDirec=" please turn left " + str(-nextangle) + " degrees"
    elif nextangle==0:
        rotationDirec=" please go straight "
    return rotationDirec
def getlength():
    length = 65535
    while length>30000:
        ser.write(longl.to_bytes(1, byteorder='big'))
        response3=ser.read()
        response4=ser.read()
        ser.write(longl.to_bytes(1, byteorder='big'))
        response1=ser.read()
        response2=ser.read()
        ser.write(ack.to_bytes(1, byteorder='big'))
        lengthl=int.from_bytes(response3, byteorder='big')*256+int.from_bytes(response4, byteorder='big')
        lengthr=int.from_bytes(response1, byteorder='big')*256+int.from_bytes(response2, byteorder='big')
        length=(lengthl+lengthr)/2
    print("length: ",length)
    return length

def locate(vertex):
    """if i==0:
        head = location
    else:
        head = shortPath[i-1]
    tail = shortPath[i]"""
    #ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)
    length=getlength()
    for i in range(len(Path)):
        if Path[i]== vertex:
            head = Path[i-1]
        tail = vertex
    a = G1.binary[head]
    b = G1.binary[tail]
    sin = (b.y-a.y)/math.sqrt((b.y-a.y)*(b.y-a.y)+(b.x-a.x)*(b.x-a.x))
    cos = (b.x-a.x)/math.sqrt((b.y-a.y)*(b.y-a.y)+(b.x-a.x)*(b.x-a.x))
    print("sin:",sin,"cos:",cos)
    x=a.x+cos*length
    y=a.y+sin*length
    return x,y,a,b

def headingget():
    #ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)
    angle=1000
    while angle>360:
        ser.write(headingCtrl.to_bytes(1, byteorder='big'))
        response1=ser.read()
        response2=ser.read()
        time.sleep(0.5)
        angle=int.from_bytes(response1, byteorder='big')+int.from_bytes(response2, byteorder='big')
        #ser.write(ack.to_bytes(1, byteorder='big'))
    print("heading angle: ",angle)
    return angle

#espeak.synth('Please input your location: ')
location = input('Please input your location: ')
#espeak.synth('Please input the direction you are heading:：')
# heading = input('Please input the direction you are heading:')
#espeak.synth('Please input your destination: ')
destination = input('Please input your destination: ')


#get heading
heading = headingget()
#print("heading: ",heading)
    
shortPath,dis = G1.dijkstra(location,destination)
Path=shortPath
rpath=list(reversed(Path))
rpath=rpath+[location]
Path=list(reversed(rpath))
print(shortPath)

"""for i in range(len(shortPath)):
    if(i==0):
        currentNodeId=location
    else:
        currentNodeId=shortPath[i-1]
    for a in G1.binary:
        if a == shortPath[i]:
            nextNodeId=a
            string = "Next reach: " + G1.binary[a].nodeName
            #espeak.synth(string)
            print(string)
            realRotateAngle=rotationAngle(G1.binary[currentNodeId],G1.binary[nextNodeId],orientation,heading)
            turnDirection=rotationDirec(realRotateAngle)
            nextDistance=vertexTest.distance(G1.binary[currentNodeId],G1.binary[nextNodeId])
            heading=(heading+realRotateAngle)%360
            string = str(turnDirection) + str(abs(realRotateAngle))
            #espeak.synth(string)
            print(string)
            string = "nextDistance: " + str(nextDistance)
            #espeak.synth(string)
            print(string)"""
i=0
num=len(shortPath)
dis1=1000000
while num:   
    """#print(num)
    #espeak.synth("input x")
    userx=int(input("input x>"))
    #espeak.synth("input y")
    usery =int(input("input y>"))
    #espeak.synth("input heading")
    userheading=int(input("input heading>"))"""
    #ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)
    userx,usery,head,tail=locate(shortPath[i])
    print(head,tail)
    print("x:",userx,"y:",usery)
    userheading=headingget()
    #print(userheading)
    a=vertexTest.Vertex("user","USER",userx,usery)
    dis=vertexTest.distance(a,G1.binary[shortPath[i]])
    if (dis<20 and num!=1) or (dis>dis1 and num!=1):
        nextangle=rotationAngle(G1.binary[shortPath[i]],G1.binary[shortPath[i+1]],orientation,userheading)
        turnDirection=rotationDirec(nextangle)       
        #userheading=(userheading+nextangle)%360
        string = "We are arriving at: " + str(G1.binary[shortPath[i]]) + ", the next station is: " + str(G1.binary[shortPath[i+1]]) + str(turnDirection)
        #espeak.synth(string)
        print(string)
        #Distance=vertexTest.distance(G1.binary[shortPath[i]],G1.binary[shortPath[i+1]])
        #print(Distance)
        shortPath.pop(i)
        #print(shortPath[i])
        num=num-1
        if nextangle>0:
            zhuan=right
            ser.write(stop.to_bytes(1, byteorder='big'))
            response=ser.readline()
            print(response)
            while int.from_bytes(response, byteorder='big')!=0x11:
                response=ser.readline()
                print(response)
                continue
            ser.write(zhuan.to_bytes(1, byteorder='big'))
            ser.write(abs(nextangle).to_bytes(1, byteorder='big'))
            response=ser.readline()
            while int.from_bytes(response, byteorder='big')!=0x11:
                response=ser.readline()
                print(response)
                continue
        elif nextangle<0:
            zhuan=left
            ser.write(stop.to_bytes(1, byteorder='big'))
            response=ser.readline()
            print(response)
            while int.from_bytes(response, byteorder='big')!=0x11:
                response=ser.readline()
                print(response)
                continue
            ser.write(zhuan.to_bytes(1, byteorder='big'))
            ser.write(abs(nextangle).to_bytes(1, byteorder='big'))
            response=ser.readline()
            print(response)
            while int.from_bytes(response, byteorder='big')!=0x11:
                response=ser.readline()
                print(response)
                continue
        ser.write(straight.to_bytes(1, byteorder='big'))
        response=ser.readline()
        print(response)
        while int.from_bytes(response, byteorder='big')!=0x11:
            response=ser.readline()
            print(response)
            continue
        ser.write(zero.to_bytes(1, byteorder='big'))
        response=ser.readline()
        print(response)
        while int.from_bytes(response, byteorder='big')!=0x11:
            response=ser.readline()
            print(response)
            continue
        dis1=1000000
    elif (dis<20 and num==1) or (dis>dis1 and num==1):
        string = "We are arriving at: " + str(G1.binary[shortPath[i]]) + "this is the final station"
        #espeak.synth(string)        
        print(string)
        shortPath.pop(i)
        #print(shortPath[i])
        num=num-1
        ser.write(stop.to_bytes(1, byteorder='big'))
        response=ser.readline()
        print(response)
        while int.from_bytes(response, byteorder='big')!=0x11:
            response=ser.readline()
            print(response)
            continue
        dis1=1000000
    else:
        string = "We will go to: the node " + str(shortPath[i]) + " which named " + str(G1.binary[shortPath[i]].nodeName) + " within: " + str(dis) + "cm"
        #espeak.synth(string)        
        print(string)
        curangle=rotationAngle(head,tail,orientation,userheading)
        #curangle=vertexTest.angle(v1,v2)
        turnDirection=rotationDirec(curangle) 
        string = str(turnDirection)
        #espeak.synth(string)        
        print(string)
        if curangle>0:
            zhuan=right
            """ser.write(stop.to_bytes(1, byteorder='big'))
            response=ser.readline()
            while int.from_bytes(response, byteorder='big')!=0x11:
                response=ser.readline()
                continue"""
            ser.write(zhuan.to_bytes(1, byteorder='big'))
            ser.write(abs(curangle).to_bytes(1, byteorder='big'))
            response=ser.readline()
            print(response)
            while int.from_bytes(response, byteorder='big')!=0x11:
                response=ser.readline()
                print(response)
                continue
        elif curangle<0:
            zhuan=left
            ser.write(stop.to_bytes(1, byteorder='big'))
            response=ser.readline()
            print(response)
            while int.from_bytes(response, byteorder='big')!=0x11:
                response=ser.readline()
                print(response)
                continue
            ser.write(zhuan.to_bytes(1, byteorder='big'))
            ser.write(abs(curangle).to_bytes(1, byteorder='big'))
            response=ser.readline()
            while int.from_bytes(response, byteorder='big')!=0x11:
                response=ser.readline()
                print(response)
                #print(response)
                continue
        ser.write(straight.to_bytes(1, byteorder='big'))
        response=ser.readline()
        print(response)
        while int.from_bytes(response, byteorder='big')!=0x11:
            response=ser.readline()
            print(response)
            continue
        dis1=dis
        
        



"""
#test
v1 =vertexTest.Vertex('1',"A",0,0)
v2 =vertexTest.Vertex('2',"B",-5,-5)
print(vertexTest.angle(v1,v2))
"""

#filename = input('please input the filename you want: ')
#file = filename + '.txt'
#f = open(str(file),'w+')
#f.write(str(jsondata))
#f.close()


