#learn how to import your own code
#need to import vertexTest.py in some ways
import vertexTest
from vertexTest import distance
from vertexTest import angle

class Graph(object):
    def __init__(self):
        self.binary = {}
        """ initializes a graph object """
        """ figure out what to store internally for a graph"""
    def __str__(self):
        return self.binary
        """ print out the vertices and edges """
        """ you are free to print out in any output format """
    def addvertex(self,nodeId,nodeName,x,y):
        self.binary[nodeId]=vertexTest.Vertex(nodeId,nodeName,x,y)

    def addedge(self,nameA,nameB):
        v1 = self.binary[nameA]
        v2 = self.binary[nameB]
        lz = distance(v1,v2)
        return v1.setitem(nameB,lz)

    def dijkstra(self,start,end):
        rend = end
        fixed=[]
        unknown={}
        flag={}
        pre={}
        path=[]
        rpath=[]
        count=0	#count the vertex
        for i in self.binary:
            count=count+1
            unknown[i]=1000000
            flag[i]=False
        fixed=fixed+[start]
        pre[start]=0
        unknown[start]=0
        flag[start]=True
        count=count-1
        curf=start
        length=0
        while count>0:
            for i in self.binary[curf].dirc:
                if flag[i]==False and unknown[curf]+self.binary[curf].dirc[i]<unknown[i]:
                    unknown[i]=unknown[curf]+self.binary[curf].dirc[i]
                    pre[i]=curf
            min=1000000
            for i in unknown:
                if flag[i]==False and unknown[i]<=min:
                    min=unknown[i]
                    curf=i
            fixed=fixed+[curf]
            flag[curf]=True
            count=count-1
        while pre[end]!=0:
            path=path+[end]
            end=pre[end]
        #path=path+[start]
        #print(fixed)S
        #for i in unknown:
            #print(unknown[i])
        rpath=list(reversed(path))
        return rpath,unknown[rend]


#G = Graph()
#G.addvertex("A",100,200)
#G.addvertex("B",400,200)
#G.addvertex("C",400,200)
#G.addedge("A","B")
#G.addedge("A","C")
#G.addvertex("C")
#G.addvertex("D")
#G.addvertex("E")
#print (G.binary["B"])
#print (str(G.binary["A"].dirc))

