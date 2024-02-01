#!/usr/bin/env pybricks-micropython

import time
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Direction, Color, Button
from pybricks.tools import wait, DataLog
from pybricks.robotics import DriveBase
from random import randint
import statistics



class MyDriveBase(DriveBase):
    def straight(self, distance) -> None:
        super().straight(distance)
        super().stop()
    
    def turn(self, angle) -> None:
        super().turn(angle)
        super().stop()
    


left_motor = Motor(Port.C, positive_direction = Direction.COUNTERCLOCKWISE, gears = None)
right_motor = Motor(Port.B, positive_direction = Direction.CLOCKWISE, gears = None)
grab_motor = Motor(Port.D, gears = None)
drop_motor = Motor(Port.A, positive_direction = Direction.COUNTERCLOCKWISE, gears = None)
left_sensor = ColorSensor(Port.S2)
right_sensor = ColorSensor(Port.S3)


data_left = DataLog('d', 'degrees', name='data_left', timestamp=False, extension='csv')
data_right = DataLog('d', 'degrees', name='data_right', timestamp=False, extension='csv')



left_positions = []
right_positions = []

graduces = [307, 615, 920 , 1216, 1530, 1820]

left_side_sensor = UltrasonicSensor(Port.S4)
right_side_sensor = UltrasonicSensor(Port.S1)
ev3 = EV3Brick()


robot = MyDriveBase(left_motor, right_motor, wheel_diameter = 62.4, axle_track = 182)




def line(speed = 50, kn: float= 0.7, kd: float= 1.5, _last_error=[0]):
    error = left_sensor.reflection() - right_sensor.reflection()
    direct = error * kn + (error - _last_error[0]) * kd
    _last_error[0] = error

    left_motor.dc(speed + direct)
    right_motor.dc(speed - direct)


def straight_line(distance, speed = 50):
    robot.reset()
    while robot.distance() <= distance:
        line(speed)
    robot.stop()


def go_to_next_x(num):
    straight_line(300*num)


def go_to_frame(num):
    robot.straight(-75-(150*num))


def get_distanse_mean(sensor):
    sum_ = 0
    for _ in range(15):
        sum_ += sensor.distance()
        time.sleep(0.01)

    return sum_ // 15


def crossroad():
    while left_sensor.reflection() > 15 or  right_sensor.reflection() > 15:
        line()
    robot.stop()

    while left_sensor.reflection() < 25 or  right_sensor.reflection() < 25:
        line()
    robot.stop()


def turn_right():
    robot.turn(90)

    
def turn_left():
    robot.turn(-90)


def nearest(values: list, one: int):
    return min(values, key=lambda n: (abs(one - n), n))

def median(sample):
    n = len(sample)
    index = n // 2

    if n % 2:
        sorted(sample)[index]

    return sum(sorted(sample)[index - 1:index + 1]) / 2


def get_position(milimetres:list, distances:list, milimetr):

    raw_distance = []
    raw_position = 0

    index = milimetres.index(nearest(milimetres, milimetr))

    for i in range(-3, 3):
        raw_distance.append(distances[index + i])

    raw_position = median(raw_distance)


    if raw_position > 500:
        position = 0
    elif 180 <= raw_position <= 360:
        position = 2
    elif raw_position < 180:
        position = 1
    else:
        position = 3
    return position    
    

def writing():

    global left_positions
    global right_positions    

    milimetres = []
    left_distances = []
    right_distances = []

    while right_sensor.reflection() > 25 or right_sensor.reflection() > 25:
        line(40)
        data_left.log(robot.distance(), left_side_sensor.distance())
        data_right.log(robot.distance(), right_side_sensor.distance())

        milimetres.append(robot.distance())
        left_distances.append(left_side_sensor.distance())
        right_distances.append(right_side_sensor.distance())



    robot.stop()

    for _ in range(6):
        right_positions.append(get_position(milimetres, right_distances, graduces[_]))
    
    for _ in range(6):
        left_positions.append(get_position(milimetres, left_distances, graduces[_]))
    
    
    print(left_positions, right_positions)
    



def get_res_position(data_pos, cube_or_ball):
    cube_or_ball = 1 if cube_or_ball == 'cube' else 2
    res_position = []

    for key, value in data_pos.items():
        if value[0][0] == cube_or_ball:
            res_position.append((key, 'r' if cube_or_ball == 1 else 'l', value[0][1]))
        if value[1][0] == cube_or_ball:
            res_position.append((key, 'l' if cube_or_ball == 1 else 'r', value[1][1])) 

    reverse = True if cube_or_ball == 1 else False
    return list(sorted(res_position, key=lambda el: el[0], reverse=reverse))


def grab(balls: str = 'l'):
    grab_motor.run(-700)
    wait(700)
    
    crossroad()
    robot.straight(-15)

    for i in range(3):
        if i != 1:
            straight_line(92)
        else:
            straight_line(80)

        grab_motor.run(900)
        wait(1300)
        grab_motor.run(-900)
        wait(1000)
        grab_motor.stop()
    
    robot.turn(180)
    
    for i in range(2):
        crossroad()
        straight_line(50)
        ev3.speaker.beep()

    straight_line(8)

    for i in range(3):
        straight_line(82)
        grab_motor.run(900)
        wait(1300)
        grab_motor.run(-900)
        wait(1000)
        grab_motor.stop()

    robot.turn(180)

    for i in range(2):
        crossroad()
        straight_line(35)
    if balls == 'l':
        turn_left()
    else:
        turn_right()


def drop(pos):
    if pos == 1:
        robot.straight(-80)
    elif pos == 2:
        robot.straight(-210)
    else:
        robot.straight(-330)
    robot.straight(10)
    
    
    drop_motor.run_angle(-160, 170)

    drop_motor.run_angle(160, 170)

    drop_motor.stop()



    while left_sensor.reflection() > 25 or right_sensor.reflection()> 25:
        robot.drive(200, 0)

    robot.stop()
    robot.straight(55)
    




def placement_cubes(pos):
    robot.straight(-100)
    robot.turn(180)
    straight_line(18)
    ev3.speaker.beep()
    wait(500)

    last_pos = 6

    dist_to_pos = 0

    for i in range(3):
        tpos = pos[i][0]

        

        dist_to_pos = 300 * (last_pos - tpos)
        straight_line(dist_to_pos)

        print("dist_to_pos :", dist_to_pos)


        set_pos = (pos[i][2] + tpos)
        print("set_pos: ",set_pos)
        if set_pos % 2 != 0 : 
            ev3.speaker.beep()
            if pos[i][1] == 'r':
                turn_left()
            else:
                turn_right()

            drop(pos[i][2])

            if pos[i][1] == 'r':
                turn_right()
            else:
                turn_left()
            
        last_pos = pos[i][0]

        wait(1000)

    
    crossroad()
    robot.turn(180)
    straight_line(265)


def placement_balls(pos):
    last_pos = 1
    dist_to_pos = 0

    for i in range(3):
        tpos = pos[i][0]

        

        dist_to_pos = 295 * (tpos - last_pos)
        straight_line(dist_to_pos)

        print("dist_to_pos :", dist_to_pos)


        set_pos = (pos[i][2] + tpos)
        print("set_pos: ",set_pos)
        if set_pos % 2 == 0 : 
            ev3.speaker.beep()
            if pos[i][1] == 'r':
                turn_left()
            else:
                turn_right()

            drop(pos[i][2])

            if pos[i][1] == 'r':
                turn_right()
            else:
                turn_left()
            
        last_pos = pos[i][0]

        wait(1000)

    robot.turn(180)
    crossroad()
    grab_motor.run(800)
    crossroad()
    robot.straight(75)



def main():
    grab('r')

    crossroad()
    straight_line(20)
    while left_sensor.reflection() > 25 or right_sensor.reflection() > 25:
        robot.drive(-150, 0)
    robot.stop()
    robot.straight(40)
    ev3.speaker.beep()
    wait(1000)
    grab_motor.stop()
    writing()
    pos = {
    x: [
        (0 if position[0] ==0 else 1 if (x + position[0]) % 2 else 2, position[0]),
        (0 if position[1] ==0 else 1 if (x + position[1]) % 2 else 2, position[1])
    ]
    for x, position in enumerate(zip(left_positions, right_positions), 1)}

    print(get_res_position(pos, 'ball'))
    print(get_res_position(pos, 'cube'))
    
    placement_cubes(get_res_position(pos, 'cube'))
    placement_balls(get_res_position(pos, 'ball'))



main()
