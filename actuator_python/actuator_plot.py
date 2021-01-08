import sys
import os
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import serial.tools.list_ports
from datetime import datetime
import argparse

parser = argparse.ArgumentParser(
    description="Runs the actuator CLI using the connected Arduino. Requires matplotlib and pyserial."
)

parser.add_argument("-p", "--port", help="Port to find Arduino connected to. If not specified, tries to find Arduino "
                                         "automatically.")
parser.add_argument("-n", "--name", help="Name for test, written in data filename. If not specified, uses current "
                                         "datetime.")

parsed_args = parser.parse_args(sys.argv[1:])
port_to_look_for = parsed_args.port
test_name = parsed_args.name

# initialize serial port
ser = serial.Serial()

ports = list(serial.tools.list_ports.comports())
arduino_port = ""
print("Serial ports:")
for p in ports:
    print(p)

if port_to_look_for is not None:
    print("Looking for ports matching '{0}'...".format(port_to_look_for))
    for p in ports:
        if port_to_look_for.lower() in p.description.lower() or port_to_look_for.lower() in p.device.lower():
            print("Found Arduino at: " + p.device)
            arduino_port = p.device
            break

if arduino_port == "":
    print("Looking for ports with 'arduino' in the name...")
    for p in ports:
        if "arduino" in p.description.lower():
            print("Found Arduino at: " + p.device)
            arduino_port = p.device
            break

if arduino_port == "":
    print("No port found. Looking for one that says 'generic'...")
    for p in ports:
        if "generic" in p.description.lower():
            print("Found Arduino at: " + p.device)
            arduino_port = p.device
            break

if arduino_port == "":
    print("Could not find Arduino. Exiting.")
    quit()

ser.port = arduino_port  # Arduino serial port
ser.baudrate = 38400
ser.timeout = 10  # timeout in seconds

try:
    ser.open()
    if ser.is_open:
        print("\nAll right, serial port now open. Configuration:\n")
        print(ser, "\n")  # print serial parameters
    else:
        print("Serial port could not open, exiting...")
        quit()
except serial.SerialException as se:
    print("Serial port failed to open, error shown below:")
    print(str(se))
    quit()

# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
x_data = []  # data indices
dist_vals = []  # distance values
curr_vals = []  # current values

# fp = open("test_data.txt", 'r')

if test_name is not None:
    suffix = test_name
else:
    now = datetime.now()
    suffix = now.strftime("%m_%d_%Y__%H_%M_%S")

slash = os.path.sep
csv_file = open("Data{0}test_{1}.csv".format(slash, suffix), 'w')
csv_file.write("Time (ms), Dist, CurrSense\n")


# This function is called periodically from FuncAnimation
def animate(i, xs, dists):
    # print("animate called: ", i) Acquire and parse data from serial port

    # Read more than one line to catch up with the arduino. readlines() only allows a single argument limiting the
    # number of bytes it will read before returning a string array with the lines. This should catch around 4 lines (
    # assuming < 15 chars each). if this is too slow, double this limit. if the limit is too high, this function will
    # block for a long time before allowing the animate function to continue. if the limit is not provided,
    # the function will block indefinitely as it just sucks up data from the arduino, which sends lines over
    # constantly. 

    lines = ser.readlines(64)  # max 64 characters

    for line in lines:
        line = line.strip()  # ascii
        # line = fp.readline().strip()
        # print(line)
        line_as_list = line.split(b'\t')
        # print(line_as_list)

        if len(line_as_list) != 3:  # Got only part of a line, skip
            return

        try:
            i = int(line_as_list[0])
            dist_float = float(line_as_list[1])
            curr_float = float(line_as_list[2])
        except ValueError as ve:
            print("Value error occurred, skipping some data:")
            print(str(ve))
            return

        # Add x and y to lists
        xs.append(i)
        dists.append(dist_float)
        curr_vals.append(curr_float)

        csv_file.write(str(i) + ", " + str(dist_float) + ", " + str(curr_float) + "\n")

    # Limit x and y lists to 20 items
    # xs = xs[-20:]
    # ys = ys[-20:]

    # Draw x and y lists
    ax.clear()
    ax.plot(xs, dists, label="Distance")
    ax.plot(xs, curr_vals, label="Current")

    # Format plot
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('Actuator Data')
    plt.ylabel('Value')
    plt.legend()
    plt.axis([1, xs[-1], 0, max(max(dists), 1)])


def printUsage():
    print("Welcome to the MSD Ice Safety Sensor vehicle debug terminal!")
    print("Available Commands:")
    print("\t(A | a) [pwm] - Set the PWM value of the actuator.")
    print("\t(R | r) [pwm] - Set the PWM value of the drill.")
    print("\t(D | d) [dir] - Set direction of actuator and drill, 0 forward else reverse.")
    print("\t(X | x) - Stop both drill and actuator.")
    print("\t(Q | q) - Quit Program.")


def commandLine():
    printUsage()
    while True:
        sys.stdout.flush()
        line = input('>>')
        if len(line) == 0:
            pass

        # quit command
        if line[0].lower() == 'q':
            break
        
        # External command, write it to serial
        s = ('q' + line + '\n\r')
        # print(s)
        ser.write(s.encode())


# Set up plot to call animate() function periodically (interval is in milliseconds)
ani = animation.FuncAnimation(fig, animate, fargs=(x_data, dist_vals), interval=100)
plt.ion()  # Interactive mode, lets plt.show() be non-blocking
plt.show()
# print(plt.isinteractive())

commandLine()  # Call command line loop. Runs until program is quit
plt.close()  # close the plot
csv_file.close()  # and the CSV file
