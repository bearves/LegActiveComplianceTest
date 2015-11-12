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
for i in range(1, 113):
    print i
    axis.append(dataArr[:,i])

print len(time)
tplt = time / 1.0

tstart = 115;
tend   = 125;

plt.subplot(3,1,1)
plt.plot(tplt, axis[81])
plt.xlim([tstart, tend])

plt.subplot(3,1,2)
for i in range(6):
    plt.plot(tplt, axis[82+i*5+2])
plt.xlim([tstart, tend])
#plt.ylim([-0.3,0.3])
plt.grid(True)

plt.subplot(3,1,3)
for i in range(6):
    plt.plot(tplt, axis[82+i*5+1])
    plt.plot(tplt, axis[82+i*5+0])
plt.ylim([0.53,0.73])
plt.xlim([tstart, tend])

plt.show()


