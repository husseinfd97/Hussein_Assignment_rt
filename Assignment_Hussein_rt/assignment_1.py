from __future__ import print_function

import time
from sr.robot import *


R = Robot()
""" instance of the class Robot"""
a_th = 2.0                                          #Closest threshold  for the control of the linear distance of silver tokens
d_th = 0.40                                         #Closest threshold  for the control of the linear distance of silver tokens
th_gd=0.75                                          #threshold of golden tokens distance 
th_ga=80                                            #threshold of golden tokens angel
th_sd=1                                             #threshold of silver tokens distance 
th_sa=75                                            #threshold of ilver tokens angel


def drive(speed, seconds):
    """
    Function for setting a linear velocity
    
    Args: speed (int): the speed of the wheels
	  seconds (int): the time interval
    """
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0

def turn(speed, seconds):
    """
    Function for setting an angular velocity
    
    Args: speed (int): the speed of the wheels
	  seconds (int): the time interval
    """
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = -speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0

def find_silver_token():
    """
    Function to find the closest silver token Returns:
	dist (float): distance of the closest silver token 
    (-1 if no silver token is detected).
	rot_y (float): angle between the robot and the silver 
    token (-1 if no silver token is detected).
    """
    dist=100
    for token in R.see():
        if token.dist < dist and token.info.marker_type is MARKER_TOKEN_SILVER:
            dist=token.dist
	    rot_y=token.rot_y
    if dist==100:
        return -1, -1
    else:
        return dist, rot_y

def find_golden_token():
    """
    Function to find the closest golden token Returns:
	dist (float): distance of the closest golden token 
    (-1 if no golden token is detected)
	rot_y (float): angle between the robot and the golden 
    token (-1 if no golden token is detected)
    """
    dist=100
    for token in R.see():
        if token.dist < dist and token.info.marker_type is MARKER_TOKEN_GOLD:
            dist=token.dist
	    rot_y=token.rot_y
    if dist==100:
        return -1, -1
    else:
        return dist, rot_y
   	
def grab_act ():
    """
    function makes the grab action which moves the silver token 
    behind and back to the first position before entering this 
    function 
    """
    turn(20, 3)
    R.release()
    turn(-20,3)


def right_left():
    """
    Function to find the nearest golden token on the right 
    and left side in cone vision within specific distance 
    and angle ranges and return the nearest left and right 
    golden token distances 
    """
    d_right=100
    d_left=100
    for i in R.see():                                   # Check in the whole tokens in the arena 
        if i.info.marker_type in (MARKER_TOKEN_GOLD):   #filter the golden tokens
            if -110<i.rot_y< -70:                       #filter the golden tokens in angle range  
                if i.dist<d_left:                       #filter the golden tokens in distance range 
                    d_left= i.dist                      #update the near godlen token then save the nearest 
    for i in R.see():
        if i.info.marker_type in (MARKER_TOKEN_GOLD):
            if 70 <i.rot_y <110:
                if i.dist<d_right:
                    d_right=i.dist



    return d_right, d_left                    


drive(50,2)
while 1:
   dist_g,rot_g=find_golden_token()
   dist_s,rot_s=find_silver_token()
   
   if dist_g < th_gd and abs(rot_g)<85:
       print ("i should stop and align")
       
       if dist_s<th_sd and abs(rot_s )< th_sa:
           print("im near to the silver")
           if -a_th<= rot_s <= a_th:                    # if the robot is well aligned with the token, we go forward
                print("Ah, I'm heading to grab")
                if dist_s<d_th:
                    R.grab()
                    grab_act()
                else:
                    drive(25, 0.25)
           elif rot_s < -a_th:                         # if the robot is not well aligned with the token, we move it on the left or on the right
                print("Left a bit more...")
                turn(-2, 0.5)
                
           elif rot_s > a_th:
                print("Right a bit more...")
                turn(+2, 0.5)
                
       else:
           near_r,near_l=right_left()
           print(near_r,near_l)
           if near_r==100:
               while(abs(rot_g)<th_ga):#loop until become far from the vision threshold 
                   dist_g,rot_g=find_golden_token()
                   turn(-5,0.25)
               drive(25,0.25)
           elif near_l==100:
                while(abs(rot_g)<th_ga):
                   dist_g,rot_g=find_golden_token()
                   turn(5,0.25)
                drive(25,0.25)
           if near_r> near_l:
               while(abs(rot_g)<th_ga):
                   dist_g,rot_g=find_golden_token()
                   turn(5,0.25)
               drive(25,0.52)
           elif near_r<near_l:
               while(abs(rot_g)<th_ga):
                   dist_g,rot_g=find_golden_token()
                   turn(-5,0.25)
               drive(25,0.25)  
   elif dist_g > th_gd:
       print ("i'm safe no gold is in range")
       if dist_s<th_sd and abs (rot_s) < th_sa:
           print("im near to the silver")
           if -a_th<= rot_s <= a_th:                  # if the robot is well aligned with the token, we go forward
                print("Ah, I'm heading to grab")
                if dist_s<d_th:
                    R.grab()
                    grab_act()

                else:
                    drive(25, 0.25)
           elif rot_s < -a_th:                       # if the robot is not well aligned with the token, we move it on the left 
                print("Left a bit more...")
                turn(-2, 0.5)
                
           elif rot_s > a_th:                        # if the robot is not well aligned with the token, we move it on the right
                print("Right a bit more...")
                turn(+2, 0.5)
                
       else:
            drive(20,0.25)
   else:
        drive(20,0.25)