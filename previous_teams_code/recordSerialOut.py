#Records the output from the given COM port and organizes it into CSVs for easy consumption by Excel
#Output to COM should be in the format [[a-z]<output>,<program time>].
#Output will be placed in the corresponding [a-z].csv file, program time should be the amount
#of milliseconds since the beginning of program execution.
from threading import Thread
import os
import serial
import matplotlib.pyplot as plt
import io
ser = serial.Serial('COM4', 38400)
dataSetCache = {}

class dataSet(object):
    #Class members
    plottingTag = None # static variable, dataset to plot
    initialized = False
    updateRequired = False #Set by the command thread when plotting tag is changed, helps avoid race condition
    logToCsv = True
    tag = None
    fp = None
    data = None
    time = None
    plot = None
    pwm = '0'
    #class functions
    #init function, runs every time anything in the class is called....
    def __init__(self, tag):
        #Only run this once, like a constructor in Java
        if(not self.initialized):
            self.initialized = True
            self.tag = tag
            self.fp = open(str(tag) + '.csv', 'a')
            self.data = []
            self.time = []
            #Default to plot the first tag you see
            if(dataSet.plottingTag == None):
                dataSet.changePlot(tag) #This is how you use static variables in Python ..... Gross
    def changePlot(tag):
        dataSet.plottingTag = tag
        dataSet.updateRequired = True
        #plt.cla()

    #adds the given line from serial out into the plot and file
    def addDataPoint(self, line):
        #Check if the command thread updated the plot, if so we need to do some processing from this thread
        if(dataSet.updateRequired):
            dataSet.updateRequired = False
            plt.title(self.tag)
            plt.clf()

        #Process line split out by Arduino
        tokens = line.split(',')
        #Store in memory for plotting and write to file for Excelling
        self.data.append(int(tokens[0][1:]))
        self.time.append(int(tokens[1]))
        if(dataSet.logToCsv and self.pwm != '0'):
            self.fp.write(dataSet.pwm + ',' + line[1:] + '\n')
            self.fp.flush()

        if(len(self.data) > 50):
            self.data = self.data[-50:]
            self.time = self.time[-50:]
            plt.clf()

        #Plotting code, only plot if you are the selected data point
        if(dataSet.plottingTag == self.tag):
            #print('Plotting: %s' % self.tag)#debug to see what is plotting
            plt.title(self.tag)
            self.plot = plt.stem(self.time, self.data, use_line_collection=True)
            plt.draw()
            plt.pause(0.01)


#Listen to the serial port and record output
#Expecting format <tag><data>,<time> with types <char><uint32_t>,<uint32_t>
def serialListener():
    listenerString = ''
    pointsCollected = 0
    #Listener loop, readLine blocks so this never exits
    while(True):

        try:
            #Parse the string character by character. Why? Because it worked
            listenerString = ser.readline().decode('utf-8').strip('\r\n')
            #print(listenerString)
            #must begin with tag, exclamation point marks a command the other thread sent, and disregard the first second
            if((not listenerString[0].isalpha()) or ('q' in listenerString)  or pointsCollected < 5):
                pass
            #Did we already create this dataset?
            elif(listenerString[0] in dataSetCache.keys()):
                #Yes it is created, go ahead and use the one we made previously
                listenerString = listenerString[:-1]#disregard newline
                data = dataSetCache.get(listenerString[0])
                data.addDataPoint(listenerString)
            else:
                #No, need to make a new dataset and store it in the cache
                listenerString = listenerString[:-1]#disregard newline
                data = dataSet(listenerString[0])
                data.addDataPoint(listenerString)
                dataSetCache[listenerString[0]] = data #100% forgot this line, this might break things
            #Reset string
            listenerString = ''
            pointsCollected += 1
        except:
            print("An exception has occurred!")


def printUsage():
    print("Welcome to the WAITS vehicle debug terminal!")
    print("Available Commands:")
    print("\t(P | p) [tag] - Switch the plot to the given data tag.")
    print("\t(R | r) [tag] - Toggle recording to CSV, defaults on.")
    #print("\t(D | d) - Enter debug mode, defaults to off.")
    #print("\t(X | x) - Exit Debug mode, continue as normal." )
    #print("\t(S | s) [leftSpeed rightSpeed] - Set the speed of each motor.")
    #print("\t(P | p) [leftPWM rightPWM] - Set the PWM for each motor.")
    print("\t(A | a) [pwm] - Set the PWM value of the actuator.")
    print("\t(D | d) [dir] - Set direction of actuator, 0 forward else reverse.")
    print("\t(X | x) - Stop both wheels and actuator (PWM and Speed are set to 0).")


def commandLine():
    printUsage()
    while(True):
        line = input('>>')
        #Check if this is the local plot switch command, or an external command to the Microcontroller
        if(line[0] == 'p' or line[0] == 'P'):
            #local command, handle it here
            tokens = line.split(' ')
            dataSet.changePlot(tokens[1])
            #Prints to local console, never touches Microcontroller
            print('Switching to plot of data with the tag %s' % tokens[1])
        elif(line[0] == 'a' or line[0] == 'A'):
            dataSet.pwm = line.split(' ')[1]
            s = ('q' + line + '\n\r')
            #print(s)
            ser.write(s.encode())
        elif(line[0] == 'r' or line[0] == 'R'):
            dataSet.logToCsv = not dataSet.logToCsv
            print("Log to CSV set to " + str(dataSet.logToCsv))
        else:
            #External command, write it to serial
            s = ('q' + line + '\n\r')
            #print(s)
            ser.write(s.encode())

        
reader = Thread(target=serialListener)
writer = Thread(target=commandLine)
reader.start()
writer.start()
