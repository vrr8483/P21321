import sys
from acconeer.exptool import clients
from acconeer.exptool import utils
from acconeer.exptool import configs
import numpy
import math

def main():
    client = clients.UARTClient("/dev/ttyUSB1")
    client.squeeze = False

    sensor_config = configs.EnvelopeServiceConfig()
    sensor_config.sensor = 1
    sensor_config.range_interval = [0.2, 1.0]
    sensor_config.profile = sensor_config.Profile.PROFILE_2
    sensor_config.hw_accelerated_average_samples = 20
    sensor_config.downsampling_factor = 2

    session_info = client.setup_session(sensor_config)

    client.start_session()
