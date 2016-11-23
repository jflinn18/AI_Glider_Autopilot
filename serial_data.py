# sudo -s
# sudo is required for the script to read from the serial port
#   However, sudo is confused when it comes to python because 
#   Anaconda is installed. So "sudo -s" just changes the shell 
#   to a root shell
#
# "exit" will get you out of the "sudo -s" and back to your 
#   original shell
#
# NOTE: Be careful, only one thing can read from the serial port
#   at one time. So you can't run this script and the the Serial
#   Monitor at the same time. 

import serial
ser = serial.Serial('/dev/ttyUSB0', 115200)
while True:
    # b'1661\t-159\t-58\t2116\t0\t-23\t20\t\r\n'
    # the '-' are negatives
    print(ser.readline())
