import sys
from acconeer.exptool import clients
from acconeer.exptool import utils
from acconeer.exptool import configs
#from acconeer.exptool.pg_process import PGProccessDiedException, PGProcess
import numpy
import math
import time
from scipy import stats

client = clients.UARTClient("/dev/ttyUSB1")
numtries = 1

def initialize():
    '''
    **********************************
    Initialize: Call once at beginning
    **********************************
    '''
    client.squeeze = False
    
    sensor_config = configs.EnvelopeServiceConfig()
    
    sensor_config.sensor = 1
    sensor_config.range_interval = [0.2, 1.0]
    sensor_config.profile = sensor_config.Profile.PROFILE_2
    sensor_config.hw_accelerated_average_samples = 20
    sensor_config.downsampling_factor = 2
    sensor_config.repetition_mode = configs.EnvelopeServiceConfig.RepetitionMode.SENSOR_DRIVEN
    sensor_config.update_rate = 10*numtries # period of 100ms

    #print(sensor_config)

    session_info = client.setup_session(sensor_config)

    #print(session_info)

    #pg_updater = PGUpdater(sensor_config, None, session_info)
    #pg_process = PGProcess(pg_updater)
    #pg_process.start()

    client.start_session()
    '''
    **********************************
    END INITIALIZE
    **********************************
    '''


def main():
    '''
    ******************************
    Main Call
    ******************************
    '''
#valuesArray = numpy.empty(numtries)
    for j in range(numtries):
        #start = time.time()
        data_info, data = client.get_next()
        #end = time.time()
        
        tempstr = " "
        #Array is within another Array, need to get address internal array
        dataArray = data[0]
        localMaxArray = [0,0]
        breakVariable = 0
        iteratorJ = 0
        iteratorI = 0
       
        # low pass filter
        alpha = 0.5
        lpf_array = dataArray
        i = 0
        while i < len(dataArray):
            lpf_array[i] = dataArray[max(0, i-1)]*alpha + (1-alpha)*dataArray[i]
            i += 1

        dataArray = lpf_array
        iteratorI = 0
        while(iteratorI < 827):
            if(iteratorJ > 20):
                localMaxArray[0] = iteratorI - iteratorJ
                break
            else:
                if(dataArray[iteratorI] >= localMaxArray[0]):
                    localMaxArray[0] = dataArray[iteratorI]
                    iteratorJ = 0
                else:
                    if(dataArray[iteratorI] > dataArray[iteratorI + 1]):
                        iteratorJ = iteratorJ + 1
            
            iteratorI = iteratorI + 1
        
#        iteratorJ = 0
#        while(iteratorJ < 40):
#            if(dataArray[iteratorI] < dataArray[iteratorI + 1]):
#                iteratorJ = iteratorJ + 1
#            else:
#                iteratorJ = 0
#            iteratorI = iteratorI + 1
#
#        iteratorJ = 0
#
#        while(iteratorI < 827):
#            if(iteratorJ > 20):
#                localMaxArray[0] = iteratorI - iteratorJ
#                break
#            else:
#                if(dataArray[iteratorI] >= localMaxArray[0]):
#                    localMaxArray[0] = dataArray[iteratorI]
#                    iteratorJ = 0
#                else:
#                    iteratorJ = iteratorJ + 1
#        
#            iteratorI = iteratorI + 1
  
      
        while(dataArray[iteratorI] >= dataArray[iteratorI + 1]):
            iteratorI = iteratorI + 1

        iteratorJ = 0

        while(iteratorI < 827):
            if(iteratorJ > 20):
                localMaxArray[1] = iteratorI - iteratorJ
                break
            else:
                if(dataArray[iteratorI] >= localMaxArray[1]):
                    localMaxArray[1] = dataArray[iteratorI]
                    iteratorJ = 0
                else:
                    iteratorJ = iteratorJ + 1
        
            iteratorI = iteratorI + 1

#print((((localMaxArray[1] - localMaxArray[0])/1.773)*0.0393701)*100)
    print(localMaxArray[1]*0.0967)
    print(localMaxArray[0]*0.0967)
    return (((localMaxArray[1] - localMaxArray[0])/1.773)*0.03807) * 255
#    modeOfArray = stats.mode(valuesArray).mode[0]
#    print(modeOfArray)
#    return modeOfArray


def disconnect():
    client.disconnect()



'''
class PGUpdater:
    def __init__(self, sensor_config, processing_config, session_info):
        self.sensor_config = sensor_config
        self.depths = utils.get_range_depths(sensor_config, session_info)

    def setup(self, win):
        win.setWindowTitle("Acconeer envelope example")

        self.plot = win.addPlot()
        self.plot.setMenuEnabled(False)
        self.plot.showGrid(x=True, y=True)
        self.plot.setLabel("bottom", "Depth (m)")
        self.plot.setLabel("left", "Amplitude")

        self.curves = []
        for i, _ in enumerate(self.sensor_config.sensor):
            curve = self.plot.plot(pen=utils.pg_pen_cycler(i))
            self.curves.append(curve)

        self.smooth_max = utils.SmoothMax(self.sensor_config.update_rate)

    def update(self, data):
        for curve, ys in zip(self.curves, data):
            curve.setData(self.depths, ys)

        self.plot.setYRange(0, self.smooth_max.update(data))


if __name__ == "__main__":
    main()
'''
