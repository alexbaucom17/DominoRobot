from marvelmind import MarvelmindHedge
from time import sleep
import matplotlib.pyplot as plt
import sys
import numpy
from threading import Thread




def update_line():

    xdata = numpy.append(plt.gca().lines[0].get_xdata(), plt.gca().lines[1].get_xdata())
    ydata = numpy.append(plt.gca().lines[0].get_ydata(), plt.gca().lines[1].get_ydata())

    pos = hedge.position()
    new_x = pos[1]
    new_y = pos[2]
    plt.gca().lines[0].set_xdata(xdata[-29:])
    plt.gca().lines[0].set_ydata(ydata[-29:])
    
    plt.gca().lines[1].set_xdata([new_x])
    plt.gca().lines[1].set_ydata([new_y])
    
    plt.draw()

def printThread():
    while True:
        try:
            sleep(3)
            pos = hedge.position()
            print (pos) # get last position and print
        except KeyboardInterrupt:
            hedge.stop()  # stop and close serial port
            sys.exit()


def main():
    
    #create plot
    global fig
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot([],[], 'ro')
    ax.grid(True)
    bx = fig.add_subplot(111)
    bx.plot([],[], 'bo')
    plt.axis('equal')
    axes = plt.gca()
    axes.set_xlim([-500,500])
    axes.set_ylim([-500,500])
    
    global hedge
    hedge = MarvelmindHedge(tty = "/dev/ttyACM0", adr=10, recieveUltrasoundPositionCallback=update_line) # create MarvelmindHedge thread
    hedge.start()
    
    plotThread = Thread(target=printThread) # create and start console out thread
    plotThread.start()
    
    plt.show()
    
main()
