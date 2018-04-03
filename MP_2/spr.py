import sys
import numpy as np
import matplotlib.pyplot as plt 


def linearfunction(p1,p2):
    if p1[0] == p2[0]:
        return None,None
    if p1[1] == p2[1]:
        a = 0
        b = p1[1]
        return a,b 
    a = (p1[1]-p2[1])/(p1[0]-p2[0])
    b = p1[1] - a * p1[0] 
    if not np.isclose(a*p2[0]+b,p2[1]) :
        print 'calculation failure during findReflexiveVertices'
        return
    return a,b

def findpn(v,polygons):
    cp = 0
    for x in range(len(polygons)):
        if v in polygons[x]:
            cp = x 
            break
    c = polygons[cp].index(v)
    p = c - 1
    n = c + 1
    if p == -1:
        p = len(polygons[cp]) -1
    if n == len(polygons[cp]):
        n = 0
    return polygons[cp][p], polygons[cp][n]


def findnearrefv(v,polygons,refv):
    cp = 0
    for x in range(len(polygons)):
        if v in polygons[x]:
            cp = x 
            break
    c = polygons[cp].index(v)
    p = c - 1
    n = c + 1
    while True:
        if p == -2 and n == -2:
            return [p_v, n_v]
        if p == -1: 
            p = len(polygons[cp])-1
        if n == len(polygons[cp]):
            n = 0
        if p != -2:
            if polygons[cp][p] in refv:
                p_v = polygons[cp][p]
                p = -2
            else:
                p = p -1
        if n != -2:
            if polygons[cp][n] in refv:
                n_v = polygons[cp][n]
                n = -2
            else:
                n = n+1

# True: There is interception
# False: No interception

def oran(A,B,C):
    return (C[1]-A[1]) * (B[0]-A[0]) >= (B[1]-A[1]) * (C[0]-A[0])

def interception(v1,v2,polygons):
    for x in range(len(polygons)):
        if True:
            for y in range(len(polygons[x])):
                if y == len(polygons[x])-1:
                    n = 0
                else:
                    n = y +1
                if not (v1 == polygons[x][y] or v1 == polygons[x][n] or v2 == polygons[x][y] or v2 == polygons[x][n]):
                    if (oran(v1,polygons[x][y],polygons[x][n]) != oran(v2,polygons[x][y],polygons[x][n])) and (oran(v1,v2,polygons[x][y]) != oran(v1,v2,polygons[x][n])):
                        return True
    return False

'''
Report reflexive vertices
'''
def findReflexiveVertices(polygons):
    vertices=[]
    for Current_poly in range(len(polygons)) :
        for Current_vector in range(len(polygons[Current_poly])) :
            #vector setup
            if Current_vector == 0 :
                Pre_v = polygons[Current_poly][len(polygons[Current_poly])-1]
            else:
                Pre_v = polygons[Current_poly][Current_vector-1]
            if Current_vector == len(polygons[Current_poly])-1:
                Next_v = polygons[Current_poly][0]
            else:
                Next_v = polygons[Current_poly][Current_vector+1]
            Current_v = polygons[Current_poly][Current_vector]
            #use Pre_v and Next_v to construct ax+b=y
            a,b = linearfunction(Next_v,Pre_v)
            #logic decision
            if Next_v[0] > Pre_v[0]:
                if Current_v[1] > a*Current_v[0]+b :
                    vertices.append(Current_v)
            elif Next_v[0] < Pre_v[0]:
                if Current_v[1] < a*Current_v[0]+b :
                    vertices.append(Current_v)  
            else:
                if Next_v[1] > Pre_v[1]:
                    if Current_v[0] < Next_v[0]:
                        vertices.append(Current_v)
                else:
                    if Current_v[0] > Next_v[0]:
                        vertices.append(Current_v)
    return vertices

'''
Compute the roadmap graph
'''

def computeSPRoadmap(polygons, reflexVertices):
    vertexMap = dict()
    adjacencyListMap = dict()
    for x in range(len(reflexVertices)):
        vertexMap[x+1] = reflexVertices[x]
    for x in range(len(vertexMap)):
        temp=[]
        main_v = vertexMap[x+1]
        for y in range(len(vertexMap)):
            if x == y: 
                continue
            picked_v = vertexMap[y+1]
            if picked_v in findnearrefv(main_v,polygons,reflexVertices):
                temp.append([y+1, np.sqrt((main_v[0]-picked_v[0])**2 +(main_v[1]-picked_v[1])**2 )])
            else:
                m_p,m_n = findpn(main_v,polygons)
                p_p,p_n = findpn(picked_v,polygons)
                a,b = linearfunction(main_v,picked_v)
                if a == None:
                    if (main_v[0] - m_p[0])*(main_v[0]-m_n[0])>=0 and (picked_v[0]-p_p[0])*(picked_v[0]-p_n[0])>=0 :
                        if not interception(main_v,picked_v,polygons):
                            temp.append([y+1, np.sqrt((main_v[0]-picked_v[0])**2 +(main_v[1]-picked_v[1])**2 )])
                else:        
                    if (a * m_p[0] + b - m_p[1])*(a * m_n[0] + b - m_n[1]) >=0 and (a * p_p[0] + b - p_p[1])*(a * p_n[0] + b - p_n[1]) >=0:
                        if not interception(main_v,picked_v,polygons):
                            temp.append([y+1, np.sqrt((main_v[0]-picked_v[0])**2 +(main_v[1]-picked_v[1])**2 )])
        adjacencyListMap[x+1] = temp
    return vertexMap, adjacencyListMap

'''
Perform uniform cost search 
'''
def getlength(path,smap):
    l = 0.0
    for x in range(len(path)-1):
        for y in range(len(smap[path[x]])):
            if path[x+1] == smap[path[x]][y][0]:
                l = l + smap[path[x]][y][1]
                break
    return l


def uniformCostSearch(adjListMap, start, goal):
    path = []
    pathLength = 0
    queue = [[start]]
    visited = [0]
    while len(queue) != 0:
        spaths = []
        #dequeue the shortest path(s) to spaths
        while True:
            spaths.append(queue.pop(0)) 
            if not (len(queue) != 0 and getlength(spaths[0],adjListMap) == getlength(queue[0],adjListMap)): break
        #check if goal is in spaths
        for x in range(len(spaths)):
            temp_v = spaths[x][len(spaths[x])-1]
            if temp_v == goal:
                queue =[]
                path = spaths[x]
                pathLength = getlength(path,adjListMap)
                return path, pathLength
            else:
                visited.append(temp_v)
                temp_list = adjListMap[temp_v]
                for y in range(len(temp_list)):
                    if not temp_list[y][0] in visited:
                        new_path = list(spaths[x])
                        new_path.append(temp_list[y][0])
                        if len(queue) == 0:
                            queue.append(new_path)
                        else:
                            for z in range(len(queue)):
                                if z == len(queue)-1:
                                    if getlength(new_path,adjListMap) <= getlength(queue[z],adjListMap):
                                        queue.insert(z,new_path)
                                    else:
                                        queue.append(new_path)
                                elif getlength(new_path,adjListMap) <= getlength(queue[z],adjListMap):
                                    queue.insert(z,new_path)
                                    break


        #add next vector to all path in spaths, and add spaths back to queue


'''
Agument roadmap to include start and goal
'''
def updateRoadmap(adjListMap, x1, y1, x2, y2,polygons,vertexMap):
    updatedALMap = adjListMap.copy()
    startLabel = 0
    goalLabel = -1
    sv = [x1,y1]
    gv = [x2,y2]
    temp0 = []
    temp1 = []
    for x in range(len(vertexMap)):
        picked_v = vertexMap[x+1]
        if not interception(sv,picked_v,polygons):
            temp0.append([x+1, np.sqrt((sv[0]-picked_v[0])**2 +(sv[1]-picked_v[1])**2 ) ])
            updatedALMap[x+1].append([0,np.sqrt((sv[0]-picked_v[0])**2 +(sv[1]-picked_v[1])**2 )])
        if not interception(gv,picked_v,polygons):
            temp1.append([x+1, np.sqrt((gv[0]-picked_v[0])**2 +(gv[1]-picked_v[1])**2 ) ])
            updatedALMap[x+1].append([-1,np.sqrt((gv[0]-picked_v[0])**2 +(gv[1]-picked_v[1])**2 )])
    if not interception(gv,sv,polygons):
    	print 'goof'
    	temp0.append([-1, np.sqrt((sv[0]-gv[0])**2 +(sv[1]-gv[1])**2 ) ])
    	temp1.append([0, np.sqrt((gv[0]-sv[0])**2 +(gv[1]-sv[1])**2 ) ])

    updatedALMap[0]  = temp0
    updatedALMap[-1] = temp1




    # Your code goes here. Note that for convenience, we 
    # let start and goal have vertex labels 0 and -1,
    # respectively. Make sure you use these as your labels
    # for the start and goal vertices in the shortest path
    # roadmap. Note that what you do here is similar to
    # when you construct the roadmap. 
    
    return startLabel, goalLabel, updatedALMap

def plotpolygon(A):
    A.append(A[0])
    x , y = zip(*A)
    plt.fill(x,y,color='grey')
    plt.plot(x,y,color='black')

def poltline(A,c):
    x , y = zip(*A)
    plt.plot(x,y,color=c)

if __name__ == "__main__":
    
    # Retrive file name for input data
    if(len(sys.argv) < 6):
        print "Five arguments required: python spr.py [env-file] [x1] [y1] [x2] [y2]"
        exit()
    
    filename = sys.argv[1]
    x1 = float(sys.argv[2])
    y1 = float(sys.argv[3])
    x2 = float(sys.argv[4])
    y2 = float(sys.argv[5])

    # Read data and parse polygons
    lines = [line.rstrip('\n') for line in open(filename)]
    polygons = []
    for line in range(0, len(lines)):
        xys = lines[line].split(';')
        polygon = []
        for p in range(0, len(xys)):
            polygon.append(map(float, xys[p].split(',')))
        polygons.append(polygon)

    # Print out the data
    print "Pologonal obstacles:"
    for p in range(0, len(polygons)):
        print str(polygons[p])
    print ""

    # Compute reflex vertices
    reflexVertices = findReflexiveVertices(polygons)
    print "Reflexive vertices:"
    print str(reflexVertices)
    print ""

    # Compute the roadmap 
    vertexMap, adjListMap = computeSPRoadmap(polygons, reflexVertices)
    print "Vertex map:"
    print str(vertexMap)
    print ""
    print "Base roadmap:"
    print str(adjListMap)
    print ""

    # Update roadmap
    start, goal, updatedALMap = updateRoadmap(adjListMap, x1, y1, x2, y2, polygons,vertexMap)
    print "Updated roadmap:"
    print str(updatedALMap)
    print ""


    # Search for a solution     
    path, length = uniformCostSearch(updatedALMap, start, goal)
    print "Final path:"
    print str(path)
    print "Final path length:" + str(length)
    
    vertexMap[start] = [x1,y1]
    vertexMap[goal] = [x2,y2]
    # figure initialization
    plt.figure(num=None, figsize=(5, 5), dpi=120, facecolor='w', edgecolor='k')
    plt.xlim(0,10)
    plt.ylim(0,10)
    plt.grid()
    #create obstacle
    for x in range(len(polygons)):
        plotpolygon(polygons[x])
    #plotmap
    for x in range(len(updatedALMap)):
        if x == (len(updatedALMap) -1) : x = -1 
        for y in range(len(updatedALMap[x])):
            poltline([vertexMap[x],vertexMap[updatedALMap[x][y][0]]],'green')
    #plot path
    for x in range(len(path)-1):
        poltline([vertexMap[path[x]],vertexMap[path[x+1]]],'red')

    plt.plot(x1,y1,'bx')
    plt.plot(x2,y2,'b*')

    plt.show()



