#!/usr/bin/python

import numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

file = open('ParsedFile.txt','r')

line = file.readline();

data = []
while line:
    nums = filter(None, line.strip('\n').split(' '))
    nums = [float(item) for item in nums]
    data.append(nums)
    line = file.readline()
print nums
file.close()
dataArr = numpy.array(data)
time = dataArr[:,0]/1000.0
axis = []
for i in range(1, 55+18):
    print i
    axis.append(dataArr[:,i])

print len(time)
tplt = time / 1.0

plt.subplot(3,1,1)

for i in range(6):
    plt.plot(tplt, axis[i*6 + 3 + 0]/1000.0)
    plt.plot(tplt, axis[i*6 + 3 + 1]/1000.0)
    plt.plot(tplt, axis[i*6 + 3 + 2]/1000.0)

plt.subplot(3,1,2)
for i in range(18):
    plt.plot(tplt, axis[i+18]/65536.0/3*2*0.005)

plt.subplot(3,1,3)
for i in range(18):
    plt.plot(tplt, axis[i+18+18]/65536.0/3*2*0.005)

plt.show()


