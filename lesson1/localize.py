# The function localize takes the following arguments:
#
# colors:
#        2D list, each entry either 'R' (for red cell) or 'G' (for green cell)
#
# measurements:
#        list of measurements taken by the robot, each entry either 'R' or 'G'
#
# motions:
#        list of actions taken by the robot, each entry of the form [dy,dx],
#        where dx refers to the change in the x-direction (positive meaning
#        movement to the right) and dy refers to the change in the y-direction
#        (positive meaning movement downward)
#        NOTE: the *first* coordinate is change in y; the *second* coordinate is
#              change in x
#
# sensor_right:
#        float between 0 and 1, giving the probability that any given
#        measurement is correct; the probability that the measurement is
#        incorrect is 1-sensor_right
#
# p_move:
#        float between 0 and 1, giving the probability that any given movement
#        command takes place; the probability that the movement command fails
#        (and the robot remains still) is 1-p_move; the robot will NOT overshoot
#        its destination in this exercise
#
# The function RETURNs (not just show or print) a 2D list (of the same
# dimensions as colors) that gives the probabilities that the robot occupies
# each cell in the world.
#
# Computation assumes the robot initially has a uniform
# probability of being in any cell.
#
# It also assumes, accoridng to the Markov Algorithm that at each step, the robot:
# 1) first makes a movement,
# 2) then takes a measurement.
#
# Motion:
#  [0,0] - stay
#  [0,1] - right
#  [0,-1] - left
#  [1,0] - down
#  [-1,0] - up


def sense(sensor_right,measurement,colors,p):

    q = [[0.0 for cols in range(len(p[0]))] for rows in range(len(p))]
    sensor_wrong = 1 - sensor_right      
    normalizer = 0.0

    #Calculate the probability distribution matrix based ont he sensory input.
    for i in range(len(p)):
        for j in range(len(p[i])):
            hit = (measurement == colors[i][j])
            q[i][j] = p[i][j] * ( (hit * sensor_right) + (1-hit) * sensor_wrong)
            normalizer += q[i][j]
    
    #Normalize the probability distribution matrix.    
    for i in range(len(p)):
        for j in range(len(p[i])):
            q[i][j] /= normalizer 
            
    return q


def move(p_move,motion,p):
    #Fill in zeros for all rows and columns first
    #[i,j] -> j = row number, i = column number

    q = [[0.0 for cols in range(len(p[0]))] for rows in range(len(p))]
    p_stay = 1 - p_move
    

    #If "i" is not zero, it means we moved up (-1) or down (1)
    if motion[0] != 0:
        for i in range(len(p)):
            for j in range(len(p[i])):
                s = p[(i- motion[0] % len(p))][j] * p_move
                s = p[i][j] * (1-p_move) + s
                q[i][j] = s 
        #print "Moved vertically."
        return q
  
    #If "j" is not zero, it means we moved left (-1) or right (1)

    elif motion[1] != 0:
        for i in range(len(p)):
            for j in range(len(p[i])):
                s = p[i][j-motion[1] % len(p[i])] * p_move
                s = p[i][j] * (1-p_move) + s
                q[i][j] = s
        return q
    
    #If both "i" and "j" are zero, no movement
    else:
        for i in range(len(p)):
            for j in range(len(p[i])):
                s = p[i][j-motion[1] % len(p[i])] * p_move
                s = p[i][j] * (1-p_move) + s
                q[i][j] = s
        
        return q


def localize(colors,measurements,motions,sensor_right,p_move):
    
    # initializes p to a uniform distribution over a grid of the same dimensions as colors
    pinit = 1.0 / (float(len(colors)) * float(len(colors[0])))
    p = [[pinit for col in range(len(colors[0]))] for row in range(len(colors))]

  
    #Using Markov localization, we MOVE, then SENSE.
    for i in range(len(measurements)):
        p = move(p_move,motions[i],p)
        p = sense(sensor_right,measurements[i],colors,p)

    return p

def show(p):
    rows = ['[' + ','.join(map(lambda x: '{0:.5f}'.format(x),r)) + ']' for r in p]
    print '[' + ',\n '.join(rows) + ']'
    

colors = [['R','G','G','R','R'],
          ['R','R','G','R','R'],
          ['R','R','G','G','R'],
          ['R','R','R','R','R']]
measurements = ['G','G','G','G','G']
motions = [[0,0],[0,1],[1,0],[1,0],[0,1]]

if len(measurements) != len(motions):
  raise ValueError, "Error in Input values for measurements and motions vectors"

p = localize(colors,measurements,motions,sensor_right = 0.7, p_move = 0.8)
show(p) # displays your answer
