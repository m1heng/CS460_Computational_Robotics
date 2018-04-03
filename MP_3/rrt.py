import sys

import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np



def poltline(A,c):
    x , y = zip(*A)
    plt.plot(x,y,color=c)

def plotpolygon(A):
    A.append(A[0])
    x , y = zip(*A)
    plt.fill(x,y,color='grey')
    plt.plot(x,y,color='black')
'''
Set up matplotlib to create a plot with an empty square
'''
def setupPlot():
    fig = plt.figure(num=None, figsize=(5, 5), dpi=120, facecolor='w', edgecolor='k')
    plt.autoscale(False)
    plt.axis('off')
    ax = fig.add_subplot(1,1,1)
    ax.set_axis_off()
    ax.add_patch(patches.Rectangle(
        (0,0),   # (x,y)
        1,          # width
        1,          # height
        fill=False
        ))
    return fig, ax

'''
Make a patch for a single pology 
'''
def createPolygonPatch(polygon, color):
    verts = []
    codes= []
    for v in range(0, len(polygon)):
        xy = polygon[v]
        verts.append((xy[0]/10., xy[1]/10.))
        if v == 0:
            codes.append(Path.MOVETO)
        else:
            codes.append(Path.LINETO)
    verts.append(verts[0])
    codes.append(Path.CLOSEPOLY)
    path = Path(verts, codes)
    patch = patches.PathPatch(path, facecolor=color, lw=1)

    return patch
    

'''
Render the problem  
'''
def drawProblem(robotStart, robotGoal, polygons):
    fig, ax = setupPlot()
    patch = createPolygonPatch(robotStart, 'green')
    ax.add_patch(patch)    
    patch = createPolygonPatch(robotGoal, 'red')
    ax.add_patch(patch)    
    for p in range(0, len(polygons)):
        patch = createPolygonPatch(polygons[p], 'gray')
        ax.add_patch(patch)    
    plt.show()


#get distance of two point 
def distance(p1,p2):
    return np.sqrt(np.sum(np.power(np.subtract(p1,p2),2)))
#get nearest point of the tree
def nst(p,newPoints):
    if len(newPoints) == 0:
        return None
    else:
        n = 1
        for x in newPoints:
            if distance(p,newPoints[x]) < distance(p,newPoints[n]):
                n = x 
    return n


#get best way to connect to rrt
def perpendicular_function(p_p,p_o):
    if p_p[1]-p_o[1] != 0 :
        a = -(p_p[0] - p_o[0])/(p_p[1]-p_o[1])
        b = p_p[1] - a * p_p[0]
        return a,b
    else:
        return None,None
def normal_function(p_p,p_o):
    if p_p[0] - p_o[0] != 0:
        a = (p_o[1]-p_p[1])/(p_o[0] - p_p[0])
        b = p_o[1] - a * p_o[0]
        return a,b
    else:
        return None,None

#input:  ab = edge, c = input_p
#output: ['distnce',(x,y), int] int = 1-> connect to a, 2->conect to b, 0->new point
def getprojection(a,b,c):
    pa_a,pa_b = perpendicular_function(a,b)
    pb_a,pb_b = perpendicular_function(b,a)
    n_a,n_b = normal_function(a,b)
    if pa_a == None:
        if b[0] <= c[0] <= a[0] or a[0] <= c[0] <= b[0] :
            out = (c[0],a[1])
            return [distance(out,c),out,0]
    elif pa_a == 0:
        if b[1] <= c[1] <= a[1] or a[1] <= c[0] <= b[0] :
            out = (a[0],c[1])
            return [distance(out,c),out, 0]
    else:
        if (c[1] - (pa_a*c[0] + pa_b)) * (c[1] - (pb_a*c[0] + pb_b)) < 0 :
            t_b = c[1] - pa_a*c[0]
            x = (n_b - t_b)/(pa_a - n_a)
            y = n_a * x + n_b
            return [distance((x,y),c), (x,y), 0]
    if distance(a,c) > distance(b,c):
    	return [distance(b,c), None, 2]
    else:
    	return [distance(a,c), None, 1]

'''
Grow a simple RRT 
'''
def growSimpleRRT(points):
    newPoints = dict()
    adjListMap = dict()
    extra = len(points)+1
    for x in points:
        if x not in newPoints:
            #first point
            if x == 1:
                adjListMap[x] = []
                newPoints[x] = points[x]
            elif x ==2:
            	adjListMap[1].append(2)
            	newPoints[2] = points[x]
            	adjListMap[2] = [1]
            else:
                # best = [distance, extra point, [a,b] or a or b ]
                best = [15, None, []]
                for nearest_p in adjListMap:
                    for pre_p in adjListMap[nearest_p]:
                        templist = getprojection(newPoints[nearest_p], newPoints[pre_p], points[x])
                        if templist[0] < best[0]:
                            best[0] = templist[0]
                            if templist[2] == 0:
                                best[1] = templist[1]
                                best[2] = [nearest_p,pre_p] 
                            elif templist[2] == 1:
                                best[1] = None
                                best[2] = [nearest_p]
                            else:
                                best[1] = None
                                best[2] = [pre_p]
                if best[1] == None:
                    newPoints[x] = points[x]
                    adjListMap[x] = [best[2][0]]
                    adjListMap[best[2][0]].append(x)
                else:
                    a = best[2][1]
                    b = best[2][0]
                    newPoints[x] = points[x]
                    newPoints[extra] = best[1]
                    adjListMap[x] = [extra]
                    adjListMap[a].remove(b)
                    adjListMap[b].remove(a)
                    adjListMap[a].append(extra)
                    adjListMap[b].append(extra)
                    adjListMap[extra] = [a,b,x]
                    extra = extra + 1
    return newPoints, adjListMap

def isMagic(robot, v1 , v2, polygons):
    update_robot1 = []
    update_robot2 = []
    for p in robot:
        update_robot1.append( (p[0] + v1[0], p[1] + v1[1]) )
        update_robot2.append( (p[0] + v2[0], p[1] + v2[1]) )
    for polygon in polygons:
        for i in range(len(polygon)):
            if i == len(polygon) -1:
                n = 0
            else:
                n = i +1
            for x in range(len(robot)):
                if isIntersect(update_robot1[x],update_robot2[x], polygon[i], polygon[n]):
                    return False
    return True
def isMeaningfule(v1,v2,maps,ps,pre_p,nearest_p):
    for i in maps:
        for y in maps[i]:
            if not ( (i == nearest_p and y == pre_p) or (y == nearest_p and i == pre_p)):
                if isIntersect(v1,v2,ps[i],ps[y]):
                    return False
    return True

def growComplexRRT(robot ,Num_point, polygons,startPoint, goalPoint): 
    Point_list = dict()
    Map = dict()
    x = 1
    extra = Num_point +3
    while x < Num_point + 3:
        if x == Num_point + 2:
            temp = startPoint
        elif x == Num_point + 1:
            temp = goalPoint
        else:
            while True:
                temp = (np.random.uniform(0,10) , np.random.uniform(0,10))
                if isCollisionFree(robot, temp, polygons) and temp[0] != 0 and temp[1] != 0:
                    points[x] = temp
                    break
        if temp not in Point_list.values():
            if x == 1:
                Map[x] = []
                Point_list[x] = temp
                x += 1
            elif x == 2:
                if isMagic(robot, Point_list[1], temp , polygons):
                    Map[1].append(2)
                    Map[2] = [1]
                    Point_list[2] = temp
                    x += 1
            else:
                best = [15, None, []]
                for a in Point_list:
                    for b in Map[a]:
                        templist = getprojection(Point_list[a], Point_list[b], temp)
                        if templist[0] < best[0] :
                            if templist[2] == 0 and isMagic(robot, templist[1], temp,polygons):
                            	best[0] = templist[0]
                                best[1] = templist[1]
                                best[2] = [a,b] 
                            elif templist[2] == 1 and isMagic(robot, Point_list[a], temp,polygons):
                            	best[0] = templist[0]
                                best[1] = None
                                best[2] = [a]
                            elif  isMagic(robot, Point_list[b], temp,polygons):
                            	best[0] = templist[0]
                                best[1] = None
                                best[2] = [b]
                if len(best[2]) == 1:
                    Point_list[x] = temp
                    Map[x] = [best[2][0]]
                    Map[best[2][0]].append(x)
                    x += 1
                elif best[0] != 15:
                    i = best[2][0]
                    j = best[2][1]
                    Point_list[x] = temp
                    Point_list[extra] = best[1]
                    Map[x] = [extra]
                    Map[i].remove(j)
                    Map[j].remove(i)
                    Map[i].append(extra)
                    Map[j].append(extra)
                    Map[extra] = [i,j,x]
                    x += 1
                    extra += 1
    return Point_list, Map

'''
Perform basic search 
'''
def basicSearch(tree, start, goal):
    path = []
    q = []
    q.append([start])
    visited = [start]
    while q:
        path = q.pop(0)
        current = path[-1]
        if current == goal:
            return path
        for next in tree.get(current,[]):
            if  not (next in visited):
                new = list(path)
                new.append(next)
                q.append(new)
                visited.append(next)
'''
Display the RRT and Path
'''
def displayRRTandPath(points, tree, path, robotStart = None, robotGoal = None, polygons = None):
    plt.figure(num=None, figsize=(5, 5), dpi=120, facecolor='w', edgecolor='k')
    plt.xlim(0,10)
    plt.ylim(0,10)
    plt.grid()
    for x in points: 
        for y in tree[x]:
            poltline( [points[x], points[y]],'black' )
    if path != None:
        for x in range(len(path)-1):
            poltline( [points[path[x]], points[path[x+1]]], 'orange' )
    if robotStart != None:
        plotpolygon(robotStart)
    if robotGoal != None:
        plotpolygon(robotGoal)
    if polygons != None:
        for polygon in polygons:
            plotpolygon(polygon)
    plt.show()
    
    return 

'''
Collision checking
'''
#Asummption 
# if in
def oran(A,B,C):
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])
def isIntersect(a1,a2,b1,b2):
    return (oran(a1,b1,b2) != oran(a2,b1,b2)) and (oran(a1,a2,b1) != oran(a1,a2,b2))

def isInside(point, obstacles):
    a = (0,0)
    b = (0,10)
    c = (10,0)
    d = (10,10)
    for obstacle in obstacles:
        temp = [0,0,0,0]
        for i in range(len(obstacle)):
            if i == len(obstacle) - 1:
                n = 0
            else:
                n = i+1
            if isIntersect(a,point,obstacle[i],obstacle[n]):
                temp[0] += 1
            if isIntersect(b,point,obstacle[i],obstacle[n]):
                temp[1] += 1
            if isIntersect(c,point,obstacle[i],obstacle[n]):
                temp[2] += 1
            if isIntersect(d,point,obstacle[i],obstacle[n]):
                temp[3] += 1
        if (temp[0] % 2 != 0) or (temp[0] % 2 != 0) or (temp[0] % 2 != 0) or (temp[0] % 2 != 0) :
            return True
    return False
    
def isCollisionFree(robot, point, obstacles):
    if isInside(point,obstacles):
        return False
    new_obstacles = list(obstacles)
    new_obstacles.append([(0,0),(0,10),(10,10),(10,0)])
    for i in range(len(robot)):
        #check if point inside obstacle
        if i == len(robot)-1:
            n = 0
        else:
            n = i +1
        v1 = (robot[i][0] + point[0] , robot[i][1] + point[1] )
        v2 = (robot[n][0] + point[0] , robot[n][1] + point[1] )
        for x in range(len(new_obstacles)):
            for y in range(len(new_obstacles[x])):
                if y == len(new_obstacles[x])-1:
                    n = 0
                else:
                    n = y +1
                if not (v1 == new_obstacles[x][y] or v1 == new_obstacles[x][n] or v2 == new_obstacles[x][y] or v2 == new_obstacles[x][n]):
                    if isIntersect(v1,v2,new_obstacles[x][y],new_obstacles[x][n]):
                        return False
    return True
    

'''
The full RRT algorithm
'''
def RRT(robot, obstacles, startPoint, goalPoint):
    if not isCollisionFree(robot,startPoint,obstacles) or not isCollisionFree(robot,goalPoint,obstacles):
        print "unvaild startPoint or goalPoint"
        return 
    print "current calculating 300 points, be patient, thanks"
    Num_point = 300
    adjListMap = dict()
    path = []
    newPoints, adjListMap = growComplexRRT(robot,Num_point, obstacles,startPoint, goalPoint)
    path = basicSearch(adjListMap, Num_point + 1, Num_point + 2)
    return newPoints, adjListMap, path


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
    robot = []
    obstacles = []
    for line in range(0, len(lines)):
        xys = lines[line].split(';')
        polygon = []
        for p in range(0, len(xys)):
            xy = xys[p].split(',')
            polygon.append((float(xy[0]), float(xy[1])))
        if line == 0 :
            robot = polygon
        else:
            obstacles.append(polygon)

    # Print out the data
    print "Robot:"
    print str(robot)
    print "Pologonal obstacles:"
    for p in range(0, len(obstacles)):
        print str(obstacles[p])
    print ""

    # Visualize
    robotStart = []
    robotGoal = []

    def start((x,y)):
        return (x+x1, y+y1)
    def goal((x,y)):
        return (x+x2, y+y2)
    robotStart = map(start, robot)
    robotGoal = map(goal, robot)
    drawProblem(robotStart, robotGoal, obstacles)

    # Example points for calling growSimpleRRT
    # You should expect many mroe points, e.g., 200-500
    points = dict()
    points[1] = (5, 5)
    points[2] = (7, 8.2)
    points[3] = (6.5, 5.2)
    points[4] = (0.3, 4)
    points[5] = (6, 3.7)
    points[6] = (9.7, 6.4)
    points[7] = (4.4, 2.8)
    points[8] = (9.1, 3.1)
    points[9] = (8.1, 6.5)
    points[10] = (0.7, 5.4)
    points[11] = (5.1, 3.9)
    points[12] = (2, 6)
    points[13] = (0.5, 6.7)
    points[14] = (8.3, 2.1)
    points[15] = (7.7, 6.3)
    points[16] = (7.9, 5)
    points[17] = (4.8, 6.1)
    points[18] = (3.2, 9.3)
    points[19] = (7.3, 5.8)
    points[20] = (9, 0.6)

    # Printing the points
    print "" 
    print "The input points are:"
    print str(points)
    print ""
    
    points, adjListMap = growSimpleRRT(points)

    # Search for a solution  
    path = basicSearch(adjListMap, 1, 20)    

    # Your visualization code 
    displayRRTandPath(points, adjListMap, path) 

    # Solve a real RRT problem
    points, adjListMap, path = RRT(robot, obstacles, (x1, y1), (x2, y2))
    
    # Your visualization code 
    displayRRTandPath(points, adjListMap, path, robotStart, robotGoal, obstacles) 




