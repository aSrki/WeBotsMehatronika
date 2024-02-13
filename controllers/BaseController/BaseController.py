from robot import MyRobot
from FSM import resetPhase, compute_velocities
import numpy as np
# from pathfinderController import PathFinderController
# from utils import transformLidarData
# import matplotlib.pyplot as plt
# from map import GridMap

if __name__ == "__main__":
    robot = MyRobot(
        x_init= 0.25, # menjas ovde pocetne koordinate i ugao
        y_init= 0.25,
        theta_ini= 0.0 # np.pi/2 == 90deg
    )
    
    # map = GridMap(3.0, 2.0, 0.05)
    
    target1 = (1.25, 0.25)
    target2 = (1.25, 1.25)
    target3 = (0.25, 1.25)
    target4 = (0.25, 0.25) # izbacis uglove
    
    targets = [target2, target4]
    
    x_ref, y_ref = targets.pop(0) # obrisao si theta

    while robot.step() != -1:
        robot.update_odom()
        
       
        v, w, done = compute_velocities(x_ref, y_ref, 0, robot.x, robot.y, robot.theta)# umesto theta si stavio 0
        if len(targets) == 0 and done:
            break
                
        if done:
            x_ref, y_ref = targets.pop(0) # obrisao si theta
            resetPhase()
        robot.setBaseVelocities(v, w)
        
        # x_s, y_s = transformLidarData(robot.x, robot.y, robot.theta, lidar_msg.ranges, lidar_msg.angles)
        
        
        # map.update_map(robot.x, robot.y, robot.theta, lidar_msg.ranges, lidar_msg.angles)
        # plt.imshow(map.grid)
        # plt.scatter(x_s, y_s)
        # plt.show(block=False)
        # plt.pause(0.001)
        
