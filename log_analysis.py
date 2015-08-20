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
for i in range(1, 43):
    print i
    axis.append(dataArr[:,i])

print len(time)
tplt = time / 1.0

plt.subplot(3,1,1)
plt.plot(tplt, axis[0]/1000.0)
plt.plot(tplt, axis[1]/1000.0)
plt.plot(tplt, axis[2]/1000.0)
plt.axis([230, 255, -120, 120])

plt.subplot(3,1,2)
for i in range(18):
    plt.plot(tplt, axis[i+6]/350.0/65536.0)
plt.axis([230, 255, 0.7, 0.9])

plt.subplot(3,1,3)
for i in range(18):
    plt.plot(tplt, axis[i+6+18]/350.0/65536.0)
plt.axis([230, 255, 0.7, 0.9])

plt.show()


