#!/usr/bin/env python

import rospy
import serial
import sys
import string
import operator
import math

from af_msgs.msg import comb
from sensor_msgs.msg import Imu



global_g = 9.8
deg_rad = math.pi / 180

def checksum(nmea_str0):
    nmea_str = nmea_str0[0:-5]
    print nmea_str
    return reduce(operator.xor, map(ord, nmea_str), 0)


port = '/dev/ttyS2'

try:
    ser = serial.Serial(port=port, baudrate=115200, timeout=1)
except serial.serialutil.SerialException:
    print("IMU not found at port " + port + ". Did you specify the correct port in the launch file?")
    # exit
    sys.exit(0)


def talker():
    ser.flush()
    global global_g, deg_rad

    pub_m = rospy.Publisher('/imu/data_raw', Imu, queue_size=100)
    pub = rospy.Publisher('gps', comb, queue_size=100)

    rospy.init_node('comb_node')
    rate = rospy.Rate(1000)  # 10hz
    
    imuMsg = Imu()
    gpsMsg = comb()

    seq = 0
    while not rospy.is_shutdown():
        if ser.read() != '$':
            continue
        line0 = ser.readline()
        print line0
        cs = line0[-4:-2]
        # print cs
        for s_tmp in line0[6:-8]:
            if s_tmp.isalpha():
                continue
        try:
            cs1 = int(cs, 16)
        except:
            continue
        cs2 = checksum(line0)
        # print("cs1,cs2", cs1, cs2)
        if cs1 != cs2:
            continue
        if line0[0:5] == "GTIMU":
            line = line0.replace("GTIMU,", "")
            line = line.replace("\r\n", "")
            if "\x00" in line:
                continue
            if not string.find('*', line):
                continue
            line = line.replace("*", ",")

            words = string.split(line, ",")
            if len(words) != 10:
                continue

            GyroX = string.atof(words[2])
            GyroY = string.atof(words[3])
            GyroZ = string.atof(words[4])
            AccX = string.atof(words[5])
            AccY = string.atof(words[6])
            AccZ = string.atof(words[7])

            imuMsg.linear_acceleration.x = AccX * global_g
            imuMsg.linear_acceleration.y = AccY * global_g
            imuMsg.linear_acceleration.z = AccZ * global_g

            imuMsg.angular_velocity.x = GyroX * deg_rad
            imuMsg.angular_velocity.y = GyroY * deg_rad
            imuMsg.angular_velocity.z = GyroZ * deg_rad

            imuMsg.header.stamp = rospy.Time.now()
            imuMsg.header.frame_id = 'imu'
            imuMsg.header.seq = seq

            seq = seq + 1
            pub_m.publish(imuMsg)
            rate.sleep()
            
        # $GPHPD, GPSWeek, GPSTime, Heading, Pitch, Track, Latitude, Longitude, Altitude, Ve , Vn, Vu,Baseline, NSV1, NSV2*cs<CR><LF>
        elif line0[0:5] == "GPHPD":
            line = line0.replace("GPHPD,", "")
            line = line.replace("\r\n", "")
            if ("\x00" in line):
                continue
            if (not string.find('*', line)):
                continue
            line = line.replace("*", ",")

            words = string.split(line, ",")
            if len(words) != 16:
                continue

            Status = words[14]

            GPSWeek = string.atoi(words[0])
            GPSTime = string.atof(words[1])
            Heading_HPD = string.atof(words[2])
            Pitch = string.atof(words[3])
            Track = string.atof(words[4])
            Latitude = string.atof(words[5])
            Longitude = string.atof(words[6])
            Altitude = string.atof(words[7])
            Ve = string.atof(words[8])
            Vn = string.atof(words[9])
            Vu = string.atof(words[10])
            Baseline = string.atof(words[11])
            NSV1 = string.atoi(words[12])
            NSV2 = string.atoi(words[13])

            gpsMsg.GPSWeek = GPSWeek
            gpsMsg.GPSTime = GPSTime
            gpsMsg.Heading = Heading_HPD
            gpsMsg.Pitch = Pitch
            gpsMsg.Roll = Track
            gpsMsg.Latitude = Latitude
            gpsMsg.Longitude = Longitude
            gpsMsg.Altitude = Altitude
            gpsMsg.Ve = Ve
            gpsMsg.Vn = Vn
            gpsMsg.Vu = Vu
            gpsMsg.Baseline = Baseline
            gpsMsg.NSV1 = NSV1
            gpsMsg.NSV2 = NSV2
            gpsMsg.Status = Status
            gpsMsg.header.stamp = rospy.Time.now()
            gpsMsg.header.frame_id = 'gps'
            gpsMsg.header.seq = seq

            pub.publish(gpsMsg)
            print("pub gps")

            seq += 1
            rate.sleep()
        else:
            continue


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

