from typing import no_type_check_decorator
from controller import Robot
import math
import numpy as np
robot = Robot() # Create robot object
timeStep = 32   # timeStep = number of milliseconds between world updates
timestep = int(robot.getBasicTimeStep())  # timestep = number of milliseconds between world updates
distanceSensors = [] # Create list to store the distance sensors
colour = robot.getCamera("colour_sensor")
colour.enable(timestep)
hole = [b';;;\xff',b'---\xff',b'ooo\xff']
swamp = [b'\x82\xd2\xf0\xff',b'\x82\xd3\xee\xff',b'\x82\xd2\xee\xff',b'\x82\xd3\xee\xff',b'\x8f\xde\xf5\xff',b'\x8e\xde\xf5\xff']
checkpoint = [b'\xff\xff\xff\xff',b'\xfe\xfe\xfe\xff',b'\xfd\xfd\xfd\xff',b'\xfc\xfc\xfc\xff',b'\xfd\xfe\xfe\xff',b'\xfc\xfb\xfb\xff']
gps = robot.getDevice("gps")
gps.enable(timestep)

# Use a for loop to enable 8 distance sensors
for i in range(5):
    distanceSensors.append(robot.getDevice("distance sensor" + str(i+1)))
    distanceSensors[i].enable(timeStep)

leftMotor = robot.getDevice("wheel2 motor")    # Motor initialization   (define)
rightMotor = robot.getDevice("wheel1 motor")

leftMotor.setPosition(float('inf'))            # Set the motors to have infinite rotation 
rightMotor.setPosition(float('inf'))

leftEncoder = leftMotor.getPositionSensor()    # Encoder initialization (define)
rightEncoder = rightMotor.getPositionSensor()

leftEncoder.enable(timeStep)                   # enable the encoders when each time world update
rightEncoder.enable(timeStep)

gyro = robot.getDevice("gyro")
gyro.enable(timeStep)
global rotation 
rotation = 0

now_array = np.array([0,4])
face_array = np.array([0,-1])#應該用compass給？！
right_turn_array = np.array([[0,1],[-1,0]])
left_turn_array = np.array([[0,-1],[1,0]]) 
u_turn_array = np.array([[-1,0],[0,-1]]) 
sequence_list = []
node_list = []
visited_list = []
wait_list = []

def moveForward(tile, velocity):
    initEn = leftEncoder.getValue()
    leftMotor.setVelocity(velocity)
    rightMotor.setVelocity(velocity)
    while robot.step(timeStep) != -1:
        if (leftEncoder.getValue() - initEn) > tile*5.81 :
            break   
def getGyro():
    diff_rotation = gyro.getValues()[1] * (timeStep / 1000)    # (fill in the update time in second)
    global rotation 
    rotation -= diff_rotation*180/math.pi
    return rotation
def getPosition(axis):
    if axis == "x":
        return gps.getValue()[0]
    if axis == "z":
        return gps.getValues()[2]

def turnR90(velocity):
    global rotation
    start_rotation = rotation
    leftMotor.setVelocity(1.0*velocity)
    rightMotor.setVelocity(-1.0*velocity)
    while robot.step(timeStep) != -1:
        turn_rotation = getGyro() - start_rotation
        #print("diff gyro value: "+ str(turn_rotation))
        if turn_rotation >= 89:
            break
def turnR180(velocity):
    global rotation
    start_rotation = rotation
    leftMotor.setVelocity(1.0*velocity)
    rightMotor.setVelocity(-1.0*velocity)
    while robot.step(timeStep) != -1:
        turn_rotation = getGyro() - start_rotation
        #print("diff gyro value: "+ str(turn_rotation))
        if turn_rotation >= 179:
            break
def turnL90(velocity):
    global rotation
    start_rotation = rotation
    leftMotor.setVelocity(-1.0*velocity)
    rightMotor.setVelocity(1.0*velocity)
    while robot.step(timeStep) != -1:
        turn_rotation = getGyro() - start_rotation
        #print("diff gyro value: "+ str(turn_rotation))
        if turn_rotation <= -89:
            break
def moveFront1(velocity):
    tileDis = 0.12
    x0 = getPosition("x")
    z0 = getPosition("z")
    leftMotor.setVelocity(velocity)
    rightMotor.setVelocity(velocity)
    while robot.step(timeStep) != -1:
        if abs(getPosition("z")-z0) or abs(getPosition("x")-x0):
            break 
def brake():
    leftMotor.setVelocity(0.0)
    rightMotor.setVelocity(0.0)
def turn(velocity, degrees):
    if degrees > 0:
        startGyro = getGyro()
        leftMotor.setVelocity(velocity)
        rightMotor.setVelocity(velocity*-1)
        while robot.step(timeStep) != -1:
            if (getGyro() - startGyro) >= degrees:
                break
    else:
        startGyro = getGyro()
        leftMotor.setVelocity(velocity*-1)
        rightMotor.setVelocity(velocity)
        while robot.step(timeStep) != -1:
            if (getGyro() - startGyro) <= degrees:
                break
def make_turn(n):
    if n == 4:
        turnR90(1.0)
    elif n == 2:
        turnL90(1.0)
    elif n == 0:
        turnR180(1.0)
    else:
        pass

def getDistance(sensorNum):
    return distanceSensors[sensorNum-1].getValue()
def getColour():
    colourValue = colour.getImage()
    if colourValue in hole:
        return "hole"
    elif colourValue in swamp:
        return "swamp"
    elif colourValue in checkpoint:
        return "checkpoint"
    else:
        return "normal"
#搜尋場地的優先順序【西，北，南，東】
#例如：當車頭朝[0,-1]N,則取車子的【4右，1前，2左】[E,N,W],後進先出wait——list，即左優先
def get_directions_in_order(face):
    directions_order = [[4,1,2],[1,4,2],[2,4,1],[2,1,4]]
    directions = [[0,-1],[1,0],[-1,0],[0,1]]
    index = directions.index(face.tolist())
    return directions_order[index]
def turn_array(direction_number):
    if direction_number==1:#Face
        return np.array([[1,0],[0,1]])
    if direction_number==2:#Left
        return np.array([[0,-1],[1,0]]) 
    if direction_number==4:#Right
        return np.array([[0,1],[-1,0]])
    if direction_number==0:#Back
        return np.array([[-1,0],[-1,0]])

# put movement code here

leftMotor.setVelocity(5.0)                     
rightMotor.setVelocity(5.0)
# 節點樹枝還沒寫，死路check後，回上一個節點
while robot.step(timeStep) != -1:
    if now_array.tolist() not in visited_list:#加入訪問過visited
        visited_list.insert(0,now_array.tolist())
    if now_array.tolist() not in sequence_list:
        sequence_list.insert(0,now_array.tolist())#加入路徑在列首
    directions_order = get_directions_in_order(face_array)#檢查那幾個方向可走
    next_to = 0
    node = 0
    for a in directions_order:#後進先出,最後一個優先
        test_array = now_array + face_array.dot(turn_array(a))
        #是否可走：
        if getDistance(a) > 0.3 and (test_array.tolist() not in visited_list):
            next_to = a
            print(next_to)#
            wait_list.insert(0,test_array.tolist())
            print("wait_list:" + str(wait_list))
            node += 1
    if node > 1:
        if len(node_list) == 0:
            node_list.insert(0,now_array.tolist())
        elif now_array.tolist() != node_list[0]:
            node_list.insert(0,now_array.tolist())    
        print("node_list:" + str(node_list))
    #若無路可走
    if node == 0:#回到前一個節點
        while now_array.tolist() != node_list[0]:
            sequence_list.pop(0)#在路徑刪除now
            print("sequence_list:" + str(sequence_list))
            move = np.array(sequence_list[0])-now_array#擠出前一個格子
            #看看move往哪一邊，把車轉向那一邊
            if np.array_equal(move, face_array.dot(right_turn_array)):
                face_array = face_array.dot(right_turn_array)
                make_turn(4)
            if np.array_equal(move, face_array.dot(left_turn_array)):
                face_array = face_array.dot(left_turn_array)
                make_turn(2)
            if np.array_equal(move, face_array.dot(u_turn_array)):
                face_array = face_array.dot(u_turn_array)
                make_turn(0)
            moveForward(1, 5.0)
            now_array = now_array + face_array
            print("back to node:"+str(now_array.tolist()))
        #回到node後，往wait走
        #move to wait[0]
        move = np.array(wait_list[0])-now_array#擠出前一個格子
        #看看move往哪一邊，把車轉向那一邊
        if np.array_equal(move, face_array.dot(right_turn_array)):
            face_array = face_array.dot(right_turn_array)
            make_turn(4)
        if np.array_equal(move, face_array.dot(left_turn_array)):
            face_array = face_array.dot(left_turn_array)
            make_turn(2)
        moveForward(1, 5.0)
        now_array = now_array + face_array
        print("back to wait:"+str(now_array.tolist()))
        wait_list.pop(0)
    else:
        make_turn(next_to)
        face_array = face_array.dot(turn_array(next_to))
        moveForward(1, 5.0)
        now_array = now_array + face_array
        print("now_array:"+str(now_array.tolist()))#
        if now_array.tolist() == wait_list[0]:
            wait_list.remove(now_array.tolist())
        print("wait_list:" + str(wait_list))
    
    if getColour() == "checkpoint":
        print(sequence_list)
        while True:
            brake()
                
             
