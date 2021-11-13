from __future__ import print_function

import time
import cmath
import math
from sr.robot import *

R = Robot()

control_tick = 0.1  #the cocstant clock cycle for control feedback     
tight_angle_th = 0.5  # for when we want as close to the angle as possible
slack_angle_th = 10 # for when we want the same heading as the angle but with greater tolerences
tight_dist_th = 0.4 # for when we want to be as close to a token as possible
slack_dist_th = 0.8 # for when we want to measure if near a token but not so close as to hit it  

max_dist_choice = 2 #limit of R.see() to get the closest tookens only
FOV_choice = 360 #field of vision of the robot 
max_dist_silv = 1 #the threshold at which the silver token is detected 
FOV_silv = 180 # field of vision detected to not detect the behind silver tokens 
angles_division = 12 #limit of spaces between two tokens is 360/thins number 

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

def move(drive , turn , seconds):
    """
    Function for setting a mixed motion velocity
    
    Args: drive (int): the linear speed
        turn (int): the rotational speed
	  seconds (int): the time interval

    (make the robot move and rotate at te same time) 
      
    """
    right_motor = drive + turn
    left_motor = drive - turn
    if abs(left_motor) > 100:
        right_motor = 100*right_motor/abs(left_motor)
        left_motor = 100 * abs(left_motor) / left_motor
    elif abs(right_motor) > 100:
        left_motor = 100*left_motor/abs(right_motor)
        right_motor = 100 * abs(right_motor) / right_motor
    R.motors[0].m0.power = right_motor
    R.motors[0].m1.power = left_motor
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0

def find_silver_token():
    """
    Function to find the closest silver token

    Returns:
	dist (float): distance of the closest silver token (-1 if no silver token is detected)
	rot_y (float): angle between the robot and the silver token (-1 if no silver token is detected)
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
    Function to find the closest golden token

    Returns:
	dist (float): distance of the closest golden token (-1 if no golden token is detected)
	rot_y (float): angle between the robot and the golden token (-1 if no golden token is detected)
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

def turn_control(input):
    K_p = 0.6
    output = input * K_p
    if abs(output) > 100:
        output = 100 * (abs(output)/output)
    return output

def speed_control(input):
    K_p = 80
    output = input * K_p
    if abs(output) > 100:
        output = 100 * (abs(output)/output)
    return output

def pos_ang(angle):# remove negative angles and angles bigger than 360
    return (angle+360)%360

def angle_from_ref(read , ref , targ):# calculate the angle to give as a feedback to angle turn using a reference angle from the 0 arena token
    read = pos_ang(read)
    ref = pos_ang(ref)
    targ = pos_ang(targ)
    output = targ - ref + read
    output = pos_ang(output)
    if output > 180:
        output = output - 360
    return output

def angle_turn(target):#uses turn control to turn a specific angle
    #r_i is reading of every control loop to find the o_i and reach the target (o_i=0)

    
    gold_dist , r_i = find_arena_token(0)
    ref = r_i
    o_i  = angle_from_ref(r_i , ref , target)
    while abs(o_i) >= tight_angle_th: #float 
        turn_val = turn_control(o_i)
        turn(turn_val , control_tick)
        
        gold_dist , r_i = find_arena_token(0)
        o_i  = angle_from_ref(r_i , ref , target)
       

def find_nearest_arena_token():# find the nearst arena token and their index
    dist=1000000
    for token in R.see():
        if token.dist < dist and token.info.marker_type is MARKER_ARENA:
            dist=token.dist
            rot_y=token.rot_y
            index = token.info.offset
    if dist==1000000:
	    return -1, -1 , -1
    else:
   	    return dist, rot_y , index

def find_arena_token(index):
    #find the location of any areana token in the arena by using their index
    """
    Function to find the arena marker with the given index

    Returns:
	dist (float): distance to the marker (-1 if no marker with given index is detected)
	rot_y (float): angle between the robot and the marker (-1 if no marker with given index is detected)
    """
    dist ,rot_y = 0 ,0
    for token in R.see():
        if token.info.offset == index and token.info.marker_type is MARKER_ARENA:
            dist=token.dist
            rot_y=token.rot_y
    if dist==0 and rot_y==0:
	    return -1, -1
    else:
   	    return dist, rot_y

def distance_drive(dist):
    #uses drive control and trangulations with arena markers to drive a certain distance
    read_dist , read_ang ,arn_ofs = find_nearest_arena_token()
    ref = read_dist * math.cos(math.radians(read_ang))
    moved = 0
    while moved+tight_dist_th < dist: 
        spd_val = speed_control((dist-moved)*1.5)
        drive(spd_val,control_tick)
        read_dist , read_ang = find_arena_token(arn_ofs)
        moved = ref - (read_dist*math.cos(math.radians(read_ang))) # dist*cos(ang)-dist*cos(ang)
        

def find_nearest_tokens(max_dist ,  FOV):# returns a list of all golden tokens in a circle with radius with max_dist and FOV(to not get the behind of the robot) 
    result=[(token.dist , token.rot_y) for token in R.see() if token.info.marker_type == MARKER_TOKEN_GOLD and token.dist<=max_dist and abs(token.rot_y)<=FOV/2]
    return result

def average(somelist):#given list of nums return the avg 
    sum = 0
    for val in somelist:
        sum = sum+val
    return sum/len(somelist)


def new_silver_token_action():
    #finds nearest silver token and goes towards it then grab it then rotate then release and return to the original heading before finding the silver token 
    silv_dist , silv_ang = find_silver_token()
    ref_ang = silv_ang
    while abs(silv_dist) >= tight_dist_th: 
        if abs(silv_ang) >= slack_angle_th:
            # time.sleep(0.5)
            angle_turn(silv_ang)
            ref_ang = ref_ang - silv_ang
        else:
            speed_val = speed_control(silv_dist)
            drive(speed_val,control_tick)
        silv_dist , silv_ang = find_silver_token()
    success = R.grab()
    while not success:
        silv_dist  , silv_ang = find_silver_token()
        time.sleep(0.5)
        angle_turn(silv_ang)
        ref_ang = ref_ang - silv_ang
        success = R.grab()
    time.sleep(0.5)
    angle_turn(178)
    ref_ang = ref_ang - 178
    R.release()
    time.sleep(0.5)
    angle_turn(ref_ang)

def new_gold_token_action():#keeps moving the robot between the golden tokens until a condition is met (silver token is near)
    

while 1:
    new_silver_token_action()
    new_gold_token_action()

