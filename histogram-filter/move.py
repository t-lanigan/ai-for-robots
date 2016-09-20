#Modify the move function to accommodate the added 
#probabilities of overshooting or undershooting 
#the intended destination.

p=[.2, .2, .2, .2, .2]
world=['green', 'gren', 'red', 'green', 'red']
measurements = ['red', 'green']
motions = [1]
pHit = 0.9
pMiss = 0.1
pExact = 1.0
#pOvershoot = 0.1
#pUndershoot = 0.1

def sense(p, Z):
    q=[]
    for i in range(len(p)):
        hit = (Z == world[i])
        q.append(p[i] * (hit * pHit + (1-hit) * pMiss))
    s = sum(q)
    for i in range(len(q)):
        q[i] = q[i] / s
    return q

def move(p, U):
    q = []
    for i in range(len(p)):
    	s = pExact * p[(i-U) % len(p)]
    	#s += pOvershoot * p[(i-U-1) % len(p)]
    	#s += pUndershoot * p[(i-U+1) % len(p)]
        q.append(s)
    return q


p = sense(p,'red')
#p = move(p, 1)


print p
