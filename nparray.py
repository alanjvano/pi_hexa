import numpy as np

x = np.array([0,3,4,5])
for i, each in enumerate(x):
    print(str(i)+" "+str(each))

y = np.zeros((3,10))
y[0] = [1,4.4,5,6,7.7,8,60,9,0,2]
for i, each in enumerate(y):
    each = np.roll(each,1)
    print(np.sort(each)[int(10/2.0)])
    print(str(i)+" "+str(each))


