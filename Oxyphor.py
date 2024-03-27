# -*- coding: utf-8 -*-
"""
Created on Tue Mar 26 17:30:25 2024

@author: schneidmo
"""


import pyvisa
import struct
import time
import numpy as np
import matplotlib.pyplot as plt
from drawnow import drawnow, figure
import keyboard
import MecademicRobot
from threading import Thread


### Establish connection to Keithley

# Get connection to Keithley:
rm = pyvisa.ResourceManager()
rm.list_resources()
# Out: ('ASRL4::INSTR', 'ASRL5::INSTR', 'ASRL8::INSTR', 'ASRL9::INSTR')

# Get Communication to both electrometers
inst1 = rm.open_resource('ASRL8::INSTR') # Beam pipe
inst1.write(":SYST:ZCH 1")
inst1.write(":SYST:ZCH 0")
inst1.write(":FUNC 'CHAR'")
inst2 = rm.open_resource('ASRL9::INSTR') # minibeam
inst2.write(":SYST:ZCH 1")
inst2.write(":SYST:ZCH 0")
inst2.write(":FUNC 'CHAR'")
inst2.query(":MEAS:CHAR?")
# Out: '-000.0045E-09NCOUL,+0000640.971549secs,+06330RDNG#\n'

inst1.query(":meas:CHAR?")
# Out: '+000.0089E-09NCOUL,+0000587.884696secs,+05799RDNG#\n'

robot = MecademicRobot.RobotController('192.168.143.205') #(172.25.181.121')


### Some definitions

# Initialize robot, move to start position
def prepareRobot():   
    robot.connect()
    robot.ActivateRobot()
    robot.home()
    robot.MovePose(250, 0, 150, 0, 90, 0)
    
# Get reading of diamond at beam pipe
def getCorrection(result, integrationTime):
    Abfrage = inst1.query(":meas:CHAR?")
    A2 = float(Abfrage.split(",")[0][0:-5])
    t1_2 = float(Abfrage.split(",")[1][0:-5])
    time.sleep(integrationTime)
    Abfrage = inst1.query(":meas:CHAR?")
    B2 = float(Abfrage.split(",")[0][0:-5])
    t2_2 = float(Abfrage.split(",")[1][0:-5])
    
    try:
        erg = (B2-A2)/(t2_2-t1_2)
    except:
        erg = 0
    result.append(erg)
    
# Get reading of diamond at robot
def getChargeParallel(integrationTime, corrThread, results):
    corrThread.start() # To synchronize timing with above function
    Abfrage = inst2.query(":meas:CHAR?")
    A1 = float(Abfrage.split(",")[0][0:-5])
    t1_1 = float(Abfrage.split(",")[1][0:-5])
    time.sleep(integrationTime)
    Abfrage = inst2.query(":meas:CHAR?")
    B1 = float(Abfrage.split(",")[0][0:-5])
    t2_1 = float(Abfrage.split(",")[1][0:-5])
    
    corrThread.join()
    
    Charge1 = (B1-A1)/(t2_1-t1_1) 
    Charge2 = results[-1]
    
    #try:
        #correctedCharge = Charge/results[-1]
    return Charge1, Charge2
    #except:
    #    correctedCharge = 0
        

# ignore
def getCharge(integrationTime):
    Abfrage = inst1.query(":meas:CHAR?")
    A1 = float(Abfrage.split(",")[0][0:-5])
    t1_1 = float(Abfrage.split(",")[1][0:-5])
    Abfrage = inst2.query(":meas:CHAR?")
    A2 = float(Abfrage.split(",")[0][0:-5])
    t1_2 = float(Abfrage.split(",")[1][0:-5])
    time.sleep(integrationTime)
    Abfrage = inst1.query(":meas:CHAR?")
    B1 = float(Abfrage.split(",")[0][0:-5])
    t2_1 = float(Abfrage.split(",")[1][0:-5])
    Abfrage = inst2.query(":meas:CHAR?")
    B2 = float(Abfrage.split(",")[0][0:-5])
    t2_2 = float(Abfrage.split(",")[1][0:-5])
    try:
        return ((B1-A1)/(t2_1-t1_1))/((B2-A2)/(t2_2-t1_2))
    except:
        return 0

def drawFigure():
    plt.plot(positions, Charge)
    

### Prepare robot for scan

robot = MecademicRobot.RobotController('192.168.143.205') #(172.25.181.121')
robot.connect()
prepareRobot()
robot.home()
robot.ResetError()


### start from here to inialize for scan with diamond

# move robot to position 
# 1st argument: horizontal position
# 2nd: in beam direction
# 3rd: vertical position
# Modify here and in main loop !!!
robot.MovePose(220, 0, 116, 0, 90, 0)#270,0,155,0,90,0 (13.12.21 microDimamond Position)

# Definition for scan with diamond
resolution = 0.05 #mm (Step size in mm)
fieldSize = 6 #mm (Scan length)
steps = int(fieldSize/resolution)
integrationTime = 1 #s
positions = []
Charge = []
results = []


### Main loop

%matplotlib qt
fhd = open('MultiSlitAperture_17.12.21_1.txt', 'w')
inst1.write(":SYST:TST:REL:RES")
inst2.write(":SYST:TST:REL:RES")
inst1.write(":SYST:ZCH 1")
inst1.write(":SYST:ZCH 0")
inst2.write(":SYST:ZCH 1")
inst2.write(":SYST:ZCH 0")
plt.figure
plt.clf()
for i in range(steps):
    robot.MovePose(268+i*resolution, 0, 116, 0, 90, 0) # match with position above
    positions.append(i*resolution)
    results.clear()
    corrThread = Thread(target=getCorrection, args=(results, integrationTime, ))
    Charge1, Charge2 = getChargeParallel(integrationTime, corrThread, results)
    Charge.append(Charge1)
    fhd.write(str(positions[-1])+ '\t'+str(Charge1)+'\t'+str(Charge2)+'\n')
    drawnow(drawFigure)
    time.sleep(0.5)
    if keyboard.is_pressed('q'):
        break
fhd.close()

robot.DeactivateRobot()
robot.disconnect()