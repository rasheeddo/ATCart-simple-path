import numpy as np
from numpy import arange, sin, cos, pi
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import struct
import sys
import time
import socket

class Plot2D():
    def __init__(self):
        self.traces = dict()
        self.t = np.arange(0,3.0,0.01)
        self.phase = 0
        # Enable antialiasing for prettier plots
        pg.setConfigOptions(antialias=True)
        #QtGui.QApplication.setGraphicsSystem('raster')
        self.app = QtGui.QApplication([])
        self.win = pg.GraphicsWindow(title="Basic plotting examples")
        self.win.resize(1000,600)
        self.win.setWindowTitle('pyqtgraph example: Plotting')

        self.world = self.win.addPlot(title="world frame")

        # Plot a ugv path
        d = 3 # target distance
        s = 0.49 # sensor offset 
        self.world.plot([0,0,d,d,0],[-s,d-s,d-s,-s,-s],pen='r')
        self.world.setXRange(-6,6,padding=0)
        self.world.setYRange(-6,6,padding=0)

        self.xu_array = np.array([],dtype=float)
        self.yu_array = np.array([],dtype=float)
        self.numPoints = 500

        self.PLOT_PORT = 24688
        self.plot_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.plot_sock.bind(('0.0.0.0', self.PLOT_PORT))
        self.plot_sock.setblocking(0)

    def start(self):
        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            QtGui.QApplication.instance().exec_()

    def set_plotdata(self,name,dataset_x,dataset_y):
        
        if name in self.traces:
            self.traces[name].setData(dataset_x,dataset_y)
        else:
            self.traces[name] = self.world.plot(pen='y',symbol='o')
            
        
        #self.world.plot([dataset_x],[dataset_y],pen=None, symbol='o')
        #self.world.setXRange(-2,2,padding=0)
        #self.world.setYRange(-2,2,padding=0)


    def update(self):
        try:
            data, addr = self.plot_sock.recvfrom(1024)
            xu,yu = struct.unpack("ff", data)
            
            if self.xu_array.size >= self.numPoints:
                self.xu_array = np.append(self.xu_array[1:self.numPoints],xu)
                self.yu_array = np.append(self.yu_array[1:self.numPoints],yu)
            else:
                self.xu_array = np.append(self.xu_array, xu)
                self.yu_array = np.append(self.yu_array, yu)


            self.set_plotdata(name='world',dataset_x=self.xu_array,dataset_y=self.yu_array)
        except socket.error:
            pass

    def animation(self):
        timer = QtCore.QTimer()
        timer.timeout.connect(self.update)
        timer.start()
        self.start()

## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    p = Plot2D()
    p.animation()