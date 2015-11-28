import time
import serial

port = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=3.0)

#move lower legs

cmd_str = ""
cmd_str += "#2P1650" 
cmd_str += "#6P1550"
cmd_str += "#10P1590"

cmd_str += "T500\r"
port.write(cmd_str)
time.sleep(0.5)

cmd_str = ""
cmd_str += "#18P1530"
cmd_str += "#22P1410"
cmd_str += "#26P1430"

cmd_str += "T500\r"
port.write(cmd_str)
time.sleep(0.5)

#move upper legs

cmd_str = ""
cmd_str += "#1P1430" 
cmd_str += "#5P1470"
cmd_str += "#9P1385"

cmd_str += "T500\r"
port.write(cmd_str)
time.sleep(0.5)

cmd_str = ""
cmd_str += "#17P1550"
cmd_str += "#21P1500"
cmd_str += "#25P1550"

cmd_str += "T500\r"
port.write(cmd_str)
time.sleep(0.5)

#move shoulders

cmd_str = ""
cmd_str += "#0P1965" 
cmd_str += "#4P1520"
cmd_str += "#8P1050"

cmd_str += "T500\r"
port.write(cmd_str)
time.sleep(0.5)

cmd_str = ""
cmd_str += "#16P1010"
cmd_str += "#20P1420"
cmd_str += "#24P1980"

cmd_str += "T500\r"
port.write(cmd_str)
time.sleep(0.5)

port.close()
