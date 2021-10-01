import numpy as np
import math

data = np.load("000123.npy")

value = 1000
cnt = 0
for d in data:
    theta = 180/math.pi*math.atan2(d[2], math.sqrt(d[0]*d[0]+d[1]*d[1]))
    if abs(theta-value)>0.1:
        value = theta
        cnt += 1
        print(theta)

print("# of channels: %d" %(cnt))