p=[0.2, 0.2, 0.2, 0.2, 0.2]
world=['green', 'green', 'red', 'green', 'red']
Z = 'red'
pHit = 0.9
pMiss = 0.1

def sense(p, Z):
	q = []

	for i in range(len(p)):
		if world[i] == Z:
			q.append(p[i] * pHit)
		else:
			q.append(p[i] * pMiss)

	total_sum = sum(q)
	newList = [i / total_sum for i in q]
	return newList


print sense(p,Z)
