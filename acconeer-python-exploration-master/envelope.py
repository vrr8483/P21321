from acconeer.exptool import clients, configs, utils
from acconeer.exptool.pg_process import PGProccessDiedException, PGProcess
import numpy
import math

def main():
    args = utils.ExampleArgumentParser().parse_args()
    utils.config_logging(args)

    if args.socket_addr:
        client = clients.SocketClient(args.socket_addr)
    elif args.spi:
        client = clients.SPIClient()
    else:
        port = args.serial_port or utils.autodetect_serial_port()
        client = clients.UARTClient(port)

    client.squeeze = False

    sensor_config = configs.EnvelopeServiceConfig()
    sensor_config.sensor = args.sensors
    sensor_config.range_interval = [0.2, 1.0]
    sensor_config.profile = sensor_config.Profile.PROFILE_2
    sensor_config.hw_accelerated_average_samples = 20
    sensor_config.downsampling_factor = 2

    session_info = client.setup_session(sensor_config)

    pg_updater = PGUpdater(sensor_config, None, session_info)
    pg_process = PGProcess(pg_updater)
    #pg_process.start()

    client.start_session()

    interrupt_handler = utils.ExampleInterruptHandler()
    print("Press Ctrl-C to end session")
    f = open("demo.txt","w");
    while not interrupt_handler.got_signal:
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
        #try:
            #pg_process.put_data(data)
        #except PGProccessDiedException:
            #break
        print(math.floor((((localMaxArray[1] - localMaxArray[0])/1.773)*0.0393701)*10)/10)
    print("Disconnecting...")
    f.close()
   #pg_process.close()
    client.disconnect()


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
