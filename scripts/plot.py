#!/usr/bin/env python
# coding=utf-8
import sys
import matplotlib.pyplot as plt

def readfile(filename):
    fp=open(filename)
    data=fp.readlines()

    x=[]
    y=[]

    for line in data:
        line_data=line.split('\t')
        x.append(float(line_data[0]))
        y.append(float(line_data[1]))
    
    return x,y

if __name__ == '__main__':
    #print('ok')
  #  x1,y1=readfile('routing_ss332.txt')
    #plt.figure('Draw')
    #p1=plt.scatter(x1,y1,c='r',marker='s',label='routing_sy')
   # plt.plot(x1,y1,'-') 
    #x2,y2 = readfile('routing.txt')
    #plt.plot(x2, y2, '-')
    x3,y3 = readfile('route.txt')
    plt.plot(x3, y3, '-')
    # x4,y4 = readfile('routing_road.txt')
    # plt.plot(x4, y4, '-')
    plt.xlabel('X')
    plt.ylabel('y')
    plt.legend()
    plt.axis('equal')
    plt.show()
    
    
