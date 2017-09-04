##### Clarification Regarding Entropy
The video mentions that entropy will decrease after the measurement update (sense) step and that entropy will increase after movement 
step (move).
In general, entropy represents the amount of uncertainty in a system. Since the measurement update step decreases uncertainty,
entropy will decrease. The movement step increases uncertainty, so entropy will increase after this step.

Let's look at our current example where the robot could be at one of five different positions. The maximum uncertainty occurs when
all positions have equal probabilities [0.2,0.2,0.2,0.2,0.2]

Following the formula Entropy=Σ(−p×log(p)), we get −5×(.2)×log(0.2)=0.699.

Taking a measurement will decrease uncertainty and entropy. Let's say after taking a measurement, the probabilities 
become [0.05,0.05,0.05,0.8,0.05]. Now we have a more certain guess as to where the robot is located and our entropy has decreased to 0.338.

#Given the list motions=[1,1] which means the robot 
#moves right and then right again, compute the posterior 
#distribution if the robot first senses red, then moves 
#right one, then senses green, then moves right again, 
#starting with a uniform prior distribution.

# A UNIFORM probability
# distribution over five grid cells, as expressed in a list of 
# five probabilities.

## Probablity is belief
p=[0.2, 0.2, 0.2, 0.2, 0.2]
world=['green', 'red', 'red', 'green', 'green']
measurements = ['red', 'green']
motions = [1,1]
pHit = 0.6
pMiss = 0.2
pExact = 0.8
pOvershoot = 0.1
pUndershoot = 0.1

def sense(p, Z):# sense is product of probablity followed by normalization
    q=[]
    for i in range(len(p)):
        hit = (Z == world[i])
        q.append(p[i] * (hit * pHit + (1-hit) * pMiss))
    s = sum(q)
    for i in range(len(q)):
        q[i] = q[i] / s #normalization of sense function
    return q

def move(p, U):
    q = []
    for i in range(len(p)):
        s = pExact * p[(i-U) % len(p)]
        s = s + pOvershoot * p[(i-U-1) % len(p)]
        s = s + pUndershoot * p[(i-U+1) % len(p)]
        q.append(s)
    return q
#
# ADD CODE HERE
#
for k in range(len(measurements)):
    p=sense(p,measurements[k])
    p=move(p,motions[k])
    
print p         
