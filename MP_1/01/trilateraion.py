import sys
import numpy as np

def trilaterate3D(distances):
    if(len(distances)!=4):
        print "Invaild Number of Input"
    r = len(distances)
    A = np.zeros((r-1, 3))
    b = np.zeros(r-1)
    for x in range(0,r-1):
        d = 0
        for y in range(0,3):
            A[x,y] = distances[x+1][y] - distances[0][y]
            d = d + A[x,y]**2
        b[x] = 0.5 * (distances[0][3]**2 - distances[x+1][3]**2 +d)

    result = np.linalg.lstsq(A,b)[0]
    result[0] = result[0] + distances[0][0]
    result[1] = result[1]+ distances[0][1]
    result[2] = result[2] + distances[0][2]

            
    return result
    #return [0.,0.,0.,0.]

if __name__ == "__main__":
    
    # Retrive file name for input data
    if(len(sys.argv) == 1):
        print "Please enter data file name."
        exit()
    
    filename = sys.argv[1]

    # Read data
    lines = [line.rstrip('\n') for line in open(filename)]
    distances = []
    for line in range(0, len(lines)):
        distances.append(map(float, lines[line].split(' ')))
    #distances = locationGenerator(random.uniform(-100.0,100.0),random.uniform(-100.0,100.0),random.uniform(-100.0,100.0))
    # Print out the data
    print "The input four points and distances, in the format of [x, y, z, d], are:"
    for p in range(0, len(distances)):
        print distances[p] 
    
    # Call the function and compute the location 
    location = trilaterate3D(distances)
    print 
    print "The location of the point is: " + str(location)
