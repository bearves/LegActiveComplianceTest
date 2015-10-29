#!/usr/bin/python

import numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

file = open('TrjFile.txt','r')

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
time = dataArr[:,0]
tplt = time / 1.0
axis = []

for i in range(1, 3):
    print i
    axis.append(dataArr[:,i])

plt.subplot(2,1, 1)
plt.plot(tplt[::], (axis[0]))
plt.grid(True)
plt.hold(False)

plt.subplot(2,1, 2)
plt.plot(tplt[:-1:], numpy.diff( axis[0])/0.001)
plt.hold(True)
plt.plot(tplt, axis[1])
plt.hold(False)
plt.grid(True)
plt.show()

#for i in range(1, 22):
    #print i
    #axis.append(dataArr[:,i])

#print len(time)
#tplt = time / 1.0

#plt.subplot(3,1, 1)
#for i in range(18):
    #plt.plot(tplt[::], (axis[i]))
    #plt.hold(True)
#plt.hold(False)
#plt.xlim([4.8, 5.4])

#plt.subplot(3,1, 2)
#for i in range(18):
    #plt.plot(tplt[:-1:], numpy.diff( (axis[i]))/0.001)
    #plt.hold(True)
#plt.xlim([4.8, 5.4])

#plt.subplot(3,1, 3)
#for i in range(18,21):
    #plt.plot(tplt[:-2:], numpy.diff(numpy.diff( (axis[i]))/0.001)/0.001)
    #plt.hold(True)
    ##plt.plot(axis[3], axis[3+2])
    ##plt.hold(True)
#plt.hold(False)
#plt.grid(True)
#plt.xlim([4.8, 5.4])
#plt.show()
