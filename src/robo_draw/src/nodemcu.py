#!/usr/bin/env python3

import rospy
import requests
from robo_draw.srv import Motion, MotionRequest
import time
import math

BASE_ADDR = 'http://192.168.45.34'
penUp = True

def callback(data: MotionRequest):
    global penUp
    # print(data)
    speed = data.twist.linear.x
    angular_speed = data.twist.angular.z
    duration = data.duration
    pen_up = data.penUp
    distance = 2 * speed * duration
    angle = -1 * angular_speed * duration * 180 / math.pi
    
    resp = {'success': False, 'message': 'Unknown error.'}

    if distance != 0:
        resp = move(distance)
    
    elif angle != 0:
        print("\rAngle:", angle.trim())
        resp = turn(angle)

    else:
        print("\rpenUp:", penUp.trim())
        penUp = pen_up
        resp = pen()
    
    print(resp+"\n")
    return resp

def move(dist):
    url = BASE_ADDR + '/move'
    data = {'distance': dist}
    resp = requests.post(url, data=data)

    if resp.status_code == 200:
        return {'success': True, 'message': 'Success!'}
    else:
        return {'success': False, 'message': resp.text}
    
def turn(angle):
    url = BASE_ADDR + '/turn'
    data = {'angle': angle}
    resp = requests.post(url, data=data)

    if resp.status_code == 200:
        return {'success': True, 'message': 'Success!'}
    else:
        return {'success': False, 'message': resp.text}

def pen():
    url = BASE_ADDR + '/pen'
    data = {'position': 'up' if penUp else 'down'}
    resp = requests.post(url, data=data)

    if resp.status_code == 200:
        return {'success': True, 'message': 'Success!'}
    else:
        return {'success': False, 'message': resp.text}
    
def service_listener():

    service = rospy.Service('motion', Motion, callback)

    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('nodemcu_comm', anonymous=True)
    while True:
        try:
            resp = requests.get(BASE_ADDR, timeout=2)
            if resp.status_code == 200:
                print("\rDevice connected at:", BASE_ADDR)
                break
            else:
                print("\rDevice not online. Retrying in 1 second.")
                print("\rResponse:", resp)
                time.sleep(1)
        except requests.exceptions.RequestException as err:
            print ("Request exception:",err)
            time.sleep(1)
    try:
        service_listener()
    except rospy.ROSInterruptException:
        pass
