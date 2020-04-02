import math
a = [0.1, 0.01, 0.001, 0.0001, -0.1, -0.01, -0.001, -0.0001]
for i in a:
    print('{}: {:.6f}'.format(i, (math.cos(2*i)-math.cos(i))/(i**2)))
