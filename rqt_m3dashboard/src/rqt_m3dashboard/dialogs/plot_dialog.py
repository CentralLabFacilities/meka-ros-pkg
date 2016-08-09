#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt4.Qt import Qt, QDialog, QPen, QBrush, QFrame, QSize, QVBoxLayout

import time
import PyQt4.Qwt5 as Qwt
from PyQt4.Qwt5.anynumpy import *

colors = [Qt.red, Qt.green, Qt.blue, Qt.yellow, Qt.cyan, Qt.magenta, Qt.darkRed, Qt.darkGreen, Qt.darkBlue, Qt.darkYellow, Qt.darkCyan, Qt.darkMagenta, Qt.black]

def resize(l, newsize, filling=None):                                                                                  
    if newsize > len(l):                                                                                 
        l.extend([filling for x in xrange(len(l), newsize)])                                                 
    else:                                                                                                
        del l[newsize:]  

class PlotDialog(QDialog):
    
    def __init__(self, parent=None):
        super(PlotDialog, self).__init__(parent)
        
        self.resize(550, 350)

        #self.verticalLayout.addWidget(self.buttonBox)
        
    def init_plot(self, component, field):
        
        self.plot = DataPlot()
        self.plot.initData(component, field)
        self.plot.resize(500, 300)
        self.plot.show()
        
        self.verticalLayout = QVBoxLayout(self)
        self.verticalLayout.addWidget(self.plot)
    
    def get_plot(self):
        return self.plot
        
    def closeEvent(self, event):
        self.reject()
        super(PlotDialog, self).closeEvent(event)
    

class DataPlot(Qwt.QwtPlot):

    def __init__(self, *args):
        Qwt.QwtPlot.__init__(self, *args)

        self.setCanvasBackground(Qt.white)
        self.alignScales()

    # __init__()
    
    def initData(self, component, field):
        # Initialize data
        self.x = arange(0.0, 100.1, 0.5)
        self.y = []
        self.curves = []

        self.setTitle("Plotting " + str(component) + " : " + str(field))
        self.insertLegend(Qwt.QwtLegend(), Qwt.QwtPlot.BottomLegend);

        self.curveR = Qwt.QwtPlotCurve("value")
        self.curveR.attach(self)
        #self.curveL = Qwt.QwtPlotCurve("Data Moving Left")
        #self.curveL.attach(self)

        #self.curveL.setSymbol(Qwt.QwtSymbol(Qwt.QwtSymbol.Ellipse,
        #                                QBrush(),
        #                                QPen(Qt.yellow),
        #                                QSize(7, 7)))

        self.curveR.setPen(QPen(Qt.red))
        #self.curveL.setPen(QPen(Qt.blue))

        mY = Qwt.QwtPlotMarker()
        mY.setLabelAlignment(Qt.AlignRight | Qt.AlignTop)
        mY.setLineStyle(Qwt.QwtPlotMarker.HLine)
        mY.setYValue(0.0)
        mY.attach(self)

        self.setAxisTitle(Qwt.QwtPlot.xBottom, "Time (seconds)")
        self.setAxisTitle(Qwt.QwtPlot.yLeft, "Values")
    
        self.timerid = self.startTimer(50)
        self.phase = 0.0
        self.currentData = []

    def setData(self, value):
        
        self.currentData = value

    def getData(self):
        return self.y

    def alignScales(self):
        self.canvas().setFrameStyle(QFrame.Box | QFrame.Plain)
        self.canvas().setLineWidth(1)
        for i in range(Qwt.QwtPlot.axisCnt):
            scaleWidget = self.axisWidget(i)
            if scaleWidget:
                scaleWidget.setMargin(0)
            scaleDraw = self.axisScaleDraw(i)
            if scaleDraw:
                scaleDraw.enableComponent(
                    Qwt.QwtAbstractScaleDraw.Backbone, False)

    # alignScales()
    
    def timerEvent(self, e):
        
        if len(self.currentData) != len(self.y):
            print "data changed"
            resize(self.y, len(self.currentData), zeros(len(self.x), Float))
            for idx, val in enumerate(self.y):
                self.curves.append(Qwt.QwtPlotCurve("value"+str(idx)))
            for idx, val in enumerate(self.curves):
                val.attach(self)
                val.setPen(QPen(colors[idx])) 
        
        if self.phase > pi - 0.0001:
            self.phase = 0.0

        for idx, val in enumerate(self.currentData):
            self.y[idx] = concatenate((self.y[idx][:1], self.y[idx][:-1]), 1)
            self.y[idx][0] = val
            self.curves[idx].setData(self.x, self.y[idx])
        
        # y moves from left to right:
        # shift y array right and assign new value y[0]
        
        
        # z moves from right to left:
        # Shift z array left and assign new value to z[n-1].
        #self.z = concatenate((self.z[1:], self.z[:1]), 1)
        #self.z[-1] = 0.8 - (2.0 * self.phase/pi) + 0.4*random.random()

        
        #self.curveL.setData(self.x, self.z)

        self.replot()
        self.phase += pi*0.02

    # timerEvent()
