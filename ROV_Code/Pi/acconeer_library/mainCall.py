import sys
from acconeer.exptool import clients
from acconeer.exptool import utils
from acconeer.exptool import configs
import numpy
import math

def main():
    data_info, data = client.get_next()
    tempstr = " "
    #Array is within another Array, need to get address internal array
    dataArray = data[0]
    localMaxArray = [0,0]
    breakVariable = 0
    iteratorJ = 0
    iteratorI = 0
    for x in dataArray:
        if(iteratorJ > 10):
            localMaxArray[0] = iteratorI - iteratorJ
            break
        else:
            if(x >= localMaxArray[0]):
                localMaxArray[0] = x
                iteratorJ = 0
            else:
                iteratorJ = iteratorJ + 1
            iteratorI = iteratorI + 1
        while(dataArray[iteratorI] >= dataArray[iteratorI + 1]):
            iteratorI = iteratorI + 1
        iteratorJ = 0
        while(iteratorI < 827):
            if(iteratorJ > 10):
                localMaxArray[1] = iteratorI - iteratorJ
                break
            else:
                if(dataArray[iteratorI] >= localMaxArray[1]):
                    localMaxArray[1] = dataArray[iteratorI]
                    iteratorJ = 0
                else:
                    iteratorJ = iteratorJ + 1
            iteratorI = iteratorI + 1
        return math.floor((((localMaxArray[1] - localMaxArray[0])/1.773)*0.0393701)*10)/10
