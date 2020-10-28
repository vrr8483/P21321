import sys

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import serial.tools.list_ports
from datetime import datetime



# initialize serial port
ser = serial.Serial()

ports = list(serial.tools.list_ports.comports())
arduino_port = ""
print("Serial ports:")
for p in ports:
    print(p)
    if "Arduino" in p.description:
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
# TODO: Allow user to name test data
now = datetime.now()
datetime_str = now.strftime("%m_%d_%Y__%H_%M_%S")
csv_file = open("test_{0}.csv".format(datetime_str), 'w')


# This function is called periodically from FuncAnimation
def animate(i, xs, dists):
    # print("animate called: ", i)
    # Acquire and parse data from serial port
    line = ser.readline().strip()  # ascii
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
    # print("\t(P | p) [tag] - Switch the plot to the given data tag.")
    # print("\t(R | r) [tag] - Toggle recording to CSV, defaults on.")
    # print("\t(D | d) - Enter debug mode, defaults to off.")
    # print("\t(X | x) - Exit Debug mode, continue as normal." )
    # print("\t(S | s) [leftSpeed rightSpeed] - Set the speed of each motor.")
    # print("\t(P | p) [leftPWM rightPWM] - Set the PWM for each motor.")
    print("\t(A | a) [pwm] - Set the PWM value of the actuator.")
    print("\t(D | d) [dir] - Set direction of actuator, 0 forward else reverse.")
    print("\t(X | x) - Stop both wheels and actuator (PWM and Speed are set to 0).")
    print("\t(Q | q) - Quit Program.")


def commandLine():
    printUsage()
    while True:
        sys.stdout.flush()
        line = input('>>')
        # Check if this is the local plot switch command, or an external command to the Microcontroller
        if line[0].lower() == 'p':
            pass
        elif line[0].lower() == 'a':
            s = ('q' + line + '\n\r')
            # print(s)
            ser.write(s.encode())
        elif line[0].lower() == 'q':
            break
        elif line[0].lower() == 'r':
            pass
        else:
            # External command, write it to serial
            s = ('q' + line + '\n\r')
            # print(s)
            ser.write(s.encode())


# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, fargs=(x_data, dist_vals), interval=80)
plt.ion()  # Interactive mode, lets plt.show() be non-blocking
plt.show()
# print(plt.isinteractive())

commandLine()  # Call command line loop. Runs until program is quit
plt.close()  # close the plot
csv_file.close()  # and the CSV file