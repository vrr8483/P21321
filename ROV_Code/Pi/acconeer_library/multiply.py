import sys
sys.path.append("/home/pi/testing/acconeer-python-exploration-master")

def multiply(a,b):
    print("Will compute", a, "times", b)
    c = 0
    for i in range(0, a):
        c = c + b
    return c
