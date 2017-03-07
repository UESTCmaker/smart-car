import urllib.request
import json
import math
import GraphTest
import vertexTest
#from espeak import espeak

string = ''

#espeak.synth('please input the building you want: ')
building = input('please input the building you want: ')
#espeak.synth('please input the level you want: ')
level = input('please input the level you want: ')
#building = 'DemoBuilding'
#level = '1'
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
        rotationDirec="please turn right "
    elif nextangle<0:
        rotationDirec="please turn left "
    elif nextangle==0:
        rotationDirec="please go straight "
    return rotationDirec
    

#espeak.synth('Please input your location: ')
location = input('Please input your location: ')
#espeak.synth('Please input the direction you are heading:：')
heading = input('Please input the direction you are heading:')
#espeak.synth('Please input your destination: ')
destination = input('Please input your destination: ')

#location = '1' 
#heading = '0'
#destination = '7'

heading=int(heading)

shortPath,dis = G1.dijkstra(location,destination)
print(shortPath)


for i in range(len(shortPath)):
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
            print(string)
i=0
num=len(shortPath)
while num:   
    #print(num)
    #espeak.synth("input x")
    userx=int(input("input x>"))
    #espeak.synth("input y")
    usery =int(input("input y>"))
    #espeak.synth("input heading")
    userheading=int(input("input heading>"))
    a=vertexTest.Vertex("user","USER",userx,usery)
    dis=vertexTest.distance(a,G1.binary[shortPath[i]])
    if dis<30 and num!=1:
        nextangle=rotationAngle(G1.binary[shortPath[i]],G1.binary[shortPath[i+1]],orientation,userheading)
        turnDirection=rotationDirec(nextangle)       
        userheading=(userheading+nextangle)%360
        string = "We are arriving at: " + str(G1.binary[shortPath[i]]) + ", the next station is: " + str(G1.binary[shortPath[i+1]]) + str(turnDirection) + str(abs(nextangle))
        #espeak.synth(string)
        print(string)
        shortPath.pop(i)
        #print(shortPath[i])
        num=num-1
    elif dis<30 and num==1:
            string = "We are arriving at: " + str(G1.binary[shortPath[i]]) + "this is the final station"
            #espeak.synth(string)        
            print(string)
            shortPath.pop(i)
            #print(shortPath[i])
            num=num-1
    else:
        string = "We will go to: the node " + str(shortPath[i]) + " which named " + str(G1.binary[shortPath[i]].nodeName) + " within: " + str(dis) + "cm"
        #espeak.synth(string)        
        print(string)
        curangle=rotationAngle(a,G1.binary[shortPath[i]],orientation,userheading)
        #curangle=vertexTest.angle(v1,v2)
        turnDirection=rotationDirec(curangle) 
        string = str(turnDirection) + str(abs(curangle))
        #espeak.synth(string)        
        print(string)

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


