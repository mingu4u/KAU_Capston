

# -*- coding: utf-8 -*
import time
import math
import pigpio
import sys
import os
import serial
import smbus
import threading
from imusensor.MPU9250 import MPU9250
from imusensor.filters import kalman
import tkinter
import sys

filename = "data"
fileformat = ".txt"
filenumber = 1

outputname = "%s%s" %(filename, fileformat)
while os.path.exists(outputname):
    outputname = "%s%d%s" %(filename, filenumber, fileformat)
    filenumber += 1
f = open(outputname,"w")

# Connect to pigpio
pi = pigpio.pi()

# Mpu9250
address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()
z2dot_prev = 0
def getmpu9250Data(dt, roll, i, tau):
    global prevgyro, z2dot_prev
    imu.readSensor()
    imu.computeOrientation()
    if i ==  0:
        roll = imu.pitch
    gyro = -imu.GyroVals[1] * 180/3.141592
    gyroLPF = tau*prevgyro + (1-tau)*gyro
    alpha = 0.99
    roll = (alpha*(roll + gyro*dt))+((1-alpha)*imu.pitch)
    Ax = imu.AccelVals[1]
    Az = imu.AccelVals[2]
    Ay = imu.AccelVals[0]
    z2dot = -Az*math.cos(roll*3.141592/180) - Ay*math.sin(roll*3.141592/180) - 9.9
    z2dot = 0.8*z2dot_prev + 0.2*z2dot
    z2dot_prev = z2dot
#    print("Ay : {0} Az : {1} z2dot : {2}".format(Ay,Az,z2dot))
    return roll, gyroLPF, gyro, z2dot

# Tfmini
ser = serial.Serial("/dev/ttyAMA0", 115200)

def getTFminiData():
    count = ser.in_waiting
    if count > 8:
        recv = ser.read(9)
        ser.reset_input_buffer()

        if recv[0] == 0x59 and recv[1] == 0x59:     #python3
            distance = (recv[2] + recv[3] * 256)/100
            #print('distance :', distance)
            ser.reset_input_buffer()
            return distance

predistance = -1
H_iterm = H_dterm_prev = prevheight_Rate = 0
height_Error_prev = 0

def disPID(CMD, distance, dt, kp1, ki1, kd1, tau2, tau):
    global predistance, H_iterm, H_dterm_prev, height_Error_prev
    distance = tau*predistance + (1-tau)*distance
    height_Error = CMD - distance
    pterm = height_Error * kp1
    dterm = kd1 * (height_Error - height_Error_prev)/dt
    height_Error_prev = height_Error
    #dterm = -kd1 * (distance - predistance)/dt
    H_iterm += ki1 * height_Error * dt
    dtermLPF = tau2 * H_dterm_prev + (1-tau2)*dterm
    H_dterm_prev = dtermLPF
    predistance = distance
    dtermLPF = max(dtermLPF, -50)
    dtermLPF = min(dtermLPF, 50)
    output = pterm + H_iterm + dtermLPF
    return output, pterm, H_iterm, dterm, dtermLPF, distance

comp_vel = 0
H_dterm_prev = 0
distance_vel_prev = 0

def disPPID(CMD, distance,z2dot, dt, kp1, kp2, ki1, kd1, tau,d_tau):
    global comp_vel, predistance, H_iterm, H_dterm_prev, distance_vel_prev
    height_Error = CMD - distance
    VelCMD = height_Error * kp1
    Vel = (distance - predistance)/dt
    distance_vel = (distance-predistance)/dt
    distance_vel = tau*distance_vel_prev + (1-tau)*(distance_vel)
    distance_vel_prev = distance_vel
    predistance = distance
    comp_vel = tau * (comp_vel + z2dot*dt) + (1-tau)*(distance_vel)
    VelError = VelCMD - comp_vel
    pterm = VelError * kp2
    H_iterm += ki1 * VelError * dt
    dterm = -kd1 * z2dot
    H_dterm_LPF = d_tau*(H_dterm_prev) + (1-d_tau)*(dterm)
    H_dterm_prev = H_dterm_LPF
    output = pterm + H_iterm + H_dterm_LPF
    output = min(output,100)
    output = max(output,-100)
    return output,VelCMD, pterm, H_iterm, dterm,comp_vel, H_dterm_LPF

#PPID
iterm1 = prevgyro = dterm_prev = 0
RateCMD = 0

def PPID(CMD, roll, gyro, dt, kp1, kp1_1, ki1, kd1, tau2):
    global iterm1, prevgyro, dterm_prev, RateCMD

    PPID_Error = CMD - roll
    RateCMD = kp1 * PPID_Error

    RateError = RateCMD - gyro
    RateDelta = gyro - prevgyro
    prevgyro = gyro

    pterm1 = kp1_1 * RateError
    iterm1 += ki1 * RateError * dt
    dterm_now = -kd1 * (RateDelta)/dt
    dtermLPF = tau2 * dterm_prev + (1-tau2) * dterm_now
    dterm_prev = dtermLPF

    iterm1 = max(iterm1, -30)
    iterm1 = min(iterm1, 30)
    dtermLPF = max(dtermLPF, -50)
    dtermLPF = min(dtermLPF, 50)
    output = pterm1 + iterm1 + dtermLPF
    output = min(output, 100)
    output = max(output, -100)
    #print(iterm1, dtermLPF)
    #print(RateError, RateDelta)
    return output, RateCMD, pterm1, iterm1, dterm_now, dtermLPF

# Motor
ESC_GPIO_L = 12 # left motor
ESC_GPIO_R = 13 # right motor

Trim_PWM = 1512

def setmotorPWM(output,houtput, angle):
    global Trim_PWM
    compensate = ((math.sqrt(1.45*9.81/(2*2.434e-05*math.cos(angle*3.1415/180)))+940.17)/0.90555-Trim_PWM)/11
    PWM_L = Trim_PWM + output + houtput + compensate
    PWM_L = min(PWM_L, 1900)
    PWM_L = max(PWM_L, 1100)
    pi.set_servo_pulsewidth(ESC_GPIO_L, PWM_L)
    PWM_R = Trim_PWM - output + houtput + compensate
    PWM_R = min(PWM_R, 1900)
    PWM_R = max(PWM_R, 1100)
    pi.set_servo_pulsewidth(ESC_GPIO_R, PWM_R)
#    print("L:{0} R:{1}".format(PWM_L, PWM_R))
#    print(compensate)
    time.sleep(0.001)
#    f.write("%d\n" %Trim_PWM)
    return PWM_L, PWM_R

def motorsetting():
    try:
        pi.set_servo_pulsewidth(ESC_GPIO_L, 0)
        pi.set_servo_pulsewidth(ESC_GPIO_R, 0)  # Maximum throttle.
        input('Press any key whenever you are ready...')

        pi.set_servo_pulsewidth(ESC_GPIO_L, 1000)
        pi.set_servo_pulsewidth(ESC_GPIO_R, 1000)# Minimum throttle.
        time.sleep(3)
        print("start")

    except KeyboardInterrupt:
        pi.set_servo_pulsewidth(ESC_GPIO_L, 0)
        pi.set_servo_pulsewidth(ESC_GPIO_R, 0)  # Stop servo pulses.
        pi.stop()  # Disconnect pigpio.
        sys.exit()

def dis_LPF(distance, tau):
    global predistance
    disLPF = tau*predistance + (1-tau)*distance
    return disLPF



def inputshaping(dt, rt, CMDinput, CMDinputprev, total_dt):
    total_dt += dt
    w = 3.141592/rt
    CMD = 0.5*(CMDinput-CMDinputprev)*(1.0-math.cos(total_dt*w))+CMDinputprev
#    print (CMD, total_dt)
    return CMD, total_dt

stop_check = 0
CMD_input = CMD_inputprev = 25.5
H_CMD_input = H_CMD_inputprev = 0.75
def CMD_input():
    global CMD_input, H_CMD_input, Trim_PWM, stop_check, total_dt, total_dt_h
    prevtime = 0
    while True:
        curtime = time.time()
        cmd_input = list(sys.stdin.readline().split())
        if cmd_input is not None:
#            print (cmd_input[1])
            if cmd_input[0] == "H":
#                H_CMD_inputprev = H_CMD_input
                H_CMD_input = float(cmd_input[1]) / 100
                total_dt_h = 0
            elif cmd_input[0] == "D":
#                CMD_inputprev = CMD_input
                CMD_input = float(cmd_input[1])
                total_dt = 0
            elif cmd_input[0] == "T":
                Trim_PWM = int(cmd_input[1])
            elif cmd_input[0] == "S":
                print("stop")
                stop_check = 1

total_dt = total_dt_h = 0

def loops():
    global CMD_input, H_CMD_input, CMD_inputprev, H_CMD_inputprev, stop_check, predistance, accel, total_dt, total_dt_h
    CMD_input = 0.0
    houput = 0
    count = roll = i = dis_dt = j = 0
    curtime = time.time()
    dis_curtime = time.time()
    rt, h_rt = 0.5, 5.0
    S_tau = 0.4
    disLPFtau = 0.0
    kp1, kp1_1, ki1, kd1, tau2 = 4.0, 0.7, 0.25, 0.01, 0.83
    H_kp1, H_kp, H_ki, H_kd, H_tau, H_D_tau =  4.0, 150.0, 30.0, 7.0, 0.98, 0.75
    deg_dterm1_LPF = 0
    while True:
        if stop_check == 1:
            if ser != None:
                ser.close()
            land = Trim_PWM
            for x in range(-10, 50, 2):
                pi.set_servo_pulsewidth(ESC_GPIO_L, land-x)
                o = 1
                pi.set_servo_pulsewidth(ESC_GPIO_R, land-x)  # Stop servo pulses.
                o = 0
                time.sleep(0.1)
            pi.set_servo_pulsewidth(ESC_GPIO_L, 1000)
            time.sleep(0.01)
            pi.set_servo_pulsewidth(ESC_GPIO_R, 1000)
            time.sleep(0.01)
            pi.stop()  # Disconnect pigpio.
            f.close()
            sys.exit()
#        f.write("%f\n" %H_CMD)
        newTime = time.time()
        dt = newTime - curtime
        curtime = newTime
        if total_dt < rt:
            CMD, total_dt = inputshaping(dt, rt, CMD_input, CMD_inputprev, total_dt)
        else:
            CMD_inputprev = CMD_input
        if total_dt_h < h_rt:
            H_CMD, total_dt_h  = inputshaping(dt, h_rt, H_CMD_input, H_CMD_inputprev, total_dt_h)
        else:
            H_CMD_inputprev = H_CMD_input
        roll, gyro, gyro_raw, z2dot = getmpu9250Data(dt, roll, i, S_tau)
        if i == 1:
            output, deg_pterm, deg_pterm1, deg_iterm1, deg_dterm1_now, deg_dterm1_LPF  = PPID(CMD, roll, gyro, dt, kp1, kp1_1, ki1, kd1, tau2)
            distance_temp = getTFminiData()
            if distance_temp is not None:
                if predistance == -1:
                    predistance = distance_temp
                distance = distance_temp
                dis_newtime = time.time()
                dis_dt = dis_newtime - dis_curtime
                dis_curtime = dis_newtime
                j = 1
            if j == 1:
                #houtput, dis_RateCMD, dis_pterm, dis_iterm, dis_dterm_now, dis_dterm_LPF, dis_rate = disPPID(H_CMD, distance, dt, H_kp1 ,H_kp, H_ki, H_kd, H_tau )
                #f.write("{0} {1} {2} {3} {4} {5} {6} {7} {8} {9} {10} {11} {12} {13} {14} {15} {16} {17} {18} {19} {20}\n".format(dt, dis_dt, roll, gyro, deg_pterm,deg_pterm1, deg_iterm1, deg_dterm1_now, deg_dterm1_LPF, distance, dis_pterm, dis_iterm, dis_dterm_now, dis_dterm_LPF, PWM_L, PWM_R, gyro_raw, CMD, H_CMD, dis_RateCMD, dis_rate))
                #distance = dis_LPF(distance, disLPFtau)
#                houtput, dis_pterm, dis_iterm, dis_dterm_now, dis_dterm_LPF, distanceLPF = disPID(H_CMD, distance, dt, H_kp, H_ki, H_kd, H_tau, disLPFtau)
                houtput,VelCMD,dis_pterm, dis_iterm,dis_dterm,comp_vel,H_dterm_LPF  = disPPID(H_CMD, distance, z2dot, dt, H_kp1, H_kp, H_ki, H_kd, H_tau,H_D_tau)
                PWM_L, PWM_R = setmotorPWM(output, houtput, roll)
                f.write("{0} {1} {2} {3} {4} {5} {6} {7} {8} {9} {10} {11} {12} {13} {14} {15} {16} {17} \n".format(dt, roll, gyro, deg_pterm,deg_pterm1, deg_iterm1, deg_dterm1_LPF, distance, VelCMD,comp_vel, dis_pterm, dis_iterm, H_dterm_LPF, PWM_L, PWM_R, gyro_raw, CMD, H_CMD))
#               f.write("{0} {1} {2} {3} {4} {5} {6} {7} {8} {9} {10} {11} {12} \n".format(dt, distance, z2dot, H_CMD,comp_vel,dis_pterm,dis_iterm,dis_dterm,houtput,VelCMD,H_dterm_LPF,PWM_L, PWM_R))
#                f.write("{0} {1} {2}\n".format(dt, z2dot, distance))
        i = 1


if __name__ == '__main__':
    motorsetting()
    if ser.is_open == False:
        ser.open()
    t_input = threading.Thread(target = CMD_input)
    t_main = threading.Thread(target = loops)
    t_main.start()
    t_input.start()

        
