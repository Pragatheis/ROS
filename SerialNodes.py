#!/usr/bin/env python

import rospy
from std_msgs.msg import String





# imports for math computations
import numpy as np

import time


import serial


class serialport_rw:

    def __init__(self):
        #Serial initialization parameters
        self.port_name = '/dev/ttyACM0'
        self.baud = 115200

        self.read_buf = ''
        self.wrt_buf = 'A+0000+0000'
        self.wrtflag = False
        

        self.ser_rate = 500
        self.serflag = False
        
        

    def wrt_cb(self,data):
        self.wrtflag = True
        self.wrt_buf = data.data;
   
    def ser_rw(self):

        rospy.init_node('serial_read_write',anonymous = True)
        pub = rospy.Publisher('ser_READ', String, queue_size = 100)
        rospy.Subscriber("ser_WRITE", String, self.wrt_cb)
        rt = rospy.Rate(self.ser_rate)
        
        try:
            ser = serial.Serial(self.port_name,self.baud)
            ser.close()
            ser.open()
            self.serflag = True
            
            time.sleep(2);
        except rospy.ROSInterruptException:
            rospy.loginfo('Error')
            
        
        beg = 0
        
        
        if self.serflag:
        # Flush the serial port once in the beginning
            if ser.inWaiting()>10 :
                rospy.loginfo(ser.read(10))    
                ser.read(62)
                rospy.loginfo('Serial port flushed')
             
            ser.write('IMU1')  
       
       
            while not rospy.is_shutdown():
            
                if self.wrtflag:
                    ser.write(self.wrt_buf)
                    self.wrtflag = False
            
                if ser.inWaiting():
                    bytes_inbuf = ser.inWaiting()
                    #print(bytes_inbuf)
                    self.read_buf = self.read_buf + ser.read(bytes_inbuf);
                # Check validity of data packet and publish the acceleration data
                    if len(self.read_buf) >= 30:
                        beg = self.read_buf.find('I')
                        #print(beg)
                        self.read_buf = self.read_buf[beg:]
                        if (len(self.read_buf)>20 and self.read_buf[19] == 'U'):
                            pub.publish(self.read_buf[0:20])
                            self.read_buf = ''
                rt.sleep()
            if rospy.is_shutdown():
                print('Sutting down')
                try:
                    
                    ser.write('IMU0')
                    print('Closing serial port')
                    ser.close()
                    
                except:
                    print('Error closing')
                
if __name__ == '__main__':
    try:
        serobj = serialport_rw()
        serobj.ser_rw()
    except rospy.ROSInterruptException:
        print("Error  in main ")
        pass 
