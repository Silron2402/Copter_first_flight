#!/usr/bin/env python
import rospy
import time
import math
from threading import Thread
from mavros_msgs.srv import CommandTOL, CommandBool, SetMode
from mavros_msgs.msg import PositionTarget, State
from geometry_msgs.msg import PoseStamped
import tf.transformations as t

# Класс для управления полетом дрона
class FlightCommander:
    def __init__(self) -> None:
        # Инициализация начальных значений
        self.current_state = None  #текущее состояние дрона
        self.pose = None           #начальная позиция 
        self._setpoint = None      #целевое положение/скорость
        self._recieved = None      #принятые данные из топика /vehicle/desPose
        #Подписываемся на топик с данными о состоянии дрона
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        #подписываемся на топик с данными о текущем положении дрона
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        #подписываемся на топик с данными о требуемом положении дрона /vehicle/desPose
        rospy.Subscriber('/vehicle/desPose', PoseStamped, self.recieved_task)
        self.rate = rospy.Rate(20)
        
        # Публикация точек назначения для дрона
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        
        #Начальные значения целевых скоростей
        self.target_Vx = 0
        self.target_Vy = 0
        self.target_Vz = 0
        self.target_yaw = 0
        self.target_yaw_rate = 0
        
        self.kp = 0.05 #коэффициент П-регулятора
        
        # Инициализация потока для отправки точек назначения
        self.setpoint_thread = None
        self.stop_thread = False
        
        # Ожидание доступности сервисов ROS
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        
    # Запуск потока для отправки точек назначения    
    def start_setpoint_thread(self):
        self.stop_thread = False
        self.setpoint_thread = Thread(target=self.send_setpoints_loop)
        self.setpoint_thread.start()

    # Остановка потока отправки точек назначения
    def stop_setpoint_thread(self):
        self.stop_thread = True
        if self.setpoint_thread:
            self.setpoint_thread.join()
            self.setpoint_thread = None

    # Цикл отправки целевого положения
    def send_setpoints_loop(self, target_rate: float=20):
        rate = rospy.Rate(target_rate)  
        while not self.stop_thread and not rospy.is_shutdown():
            if self._setpoint is None:
                continue
            if self.current_state.armed == False:
                self.arm_vehicle
            self.setpoint_pub.publish(self._setpoint)
            rate.sleep()
        self._setpoint = None
    
    # Обратный вызов для обновления текущего состояния дрона
    def state_callback(self, msg):
        self.current_state = msg
        # if msg.mode == "OFFBOARD" and self.setpoint_thread is None:
        #     self.start_setpoint_thread()
        if msg.mode != "OFFBOARD" and self.setpoint_thread is not None:
            self.stop_setpoint_thread()
    
    # Обратный вызов для обновления текущей позиции дрона    
    def pose_callback(self, msg):
        self.pose = msg
    
    #обработка данных топика /vehicle/desPose
    def recieved_task(self, msg):
        self._recieved = msg  
        if (not self._recieved is None) and (not self.pose is None):
            #расчет ошибки по положению
            err_x = self._recieved.pose.position.x - self.pose.pose.position.x
            err_y = self._recieved.pose.position.y - self.pose.pose.position.y
            err_z = self._recieved.pose.position.z - self.pose.pose.position.z
            q = self.pose.pose.orientation.x, self.pose.pose.orientation.y,self.pose.pose.orientation.z,self.pose.pose.orientation.w
            roll, pitch, yaw = t.euler_from_quaternion(q)
            print(yaw)
            self.w = self.pose.pose.orientation.w
            self.target_yaw = math.atan2(self._recieved.pose.position.x,self._recieved.pose.position.y)
            err_yaw = self.target_yaw - yaw
            print(f'Целевой угол рыскания {self.target_yaw}') 
            print(f'Фактический угол рыскания {yaw}') 
            print(f'текущая координата x {self.pose.pose.position.y}') 
            print(f'текущая координата y {self.pose.pose.position.y}') 
            print(f'текущая координата z {self.pose.pose.position.z}') 
            print(f'требуемая координата z {self._recieved.pose.position.z}') 
            print(f'текущая ошибка по координате x {err_x}') 
            print(f'текущая ошибка по координате y {err_y}') 
            print(f'текущая ошибка по координате z {err_z}') 
           #целевые скорости в абсолютных координатах
            V_x_target = err_x * 1
            V_y_target = err_y * 1
            V_z_target = err_z * 0.08
            #Целевые скорости в локальных координатах
        
            self.target_Vx = V_x_target * math.cos(yaw) +  V_y_target * math.sin(yaw)#err_x * self.kp
            self.target_Vy = -V_x_target * math.sin(yaw) + V_y_target * math.cos(yaw)#err_y * self.kp
            self.target_Vz = err_z * 0.08
            self.target_yaw_rate =  err_yaw * 0.01

        #except rospy.ServiceException as e:   
            #print(f'желаемая позиция x {self.tack_pose.pose.position.x}')
        #print(self.pose.pose.pose.position.x) 
     
        
    def land_vehicle(self):
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
            
            # The parameters for the CommandTOL service are:
            # min_pitch, yaw, latitude, longitude, altitude
            # Setting latitude, longitude, and altitude to 0 makes the drone land at its current position.
            land_response = land_service(0, 0, 0, 0, 0)
            
            if land_response.success:
                print("Landing command sent successfully!")
                return True
            else:
                print("Failed to send landing command.")
                return False
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return False
    
    def do_arm(self):
        while (self.current_state is None):
            pass
        if not self.current_state.armed:
            if self.arm_vehicle():
                print("Vehicle armed successfully!")
            else:
                print("Failed to arm the vehicle.")
                return    
    
    def do_takeoff(self, altitude: float=1.0):
        
        while (self.current_state is None):
            pass
        
        if not self.current_state.armed:
            if self.arm_vehicle():
                print("Vehicle armed successfully!")
            else:
                print("Failed to arm the vehicle.")
                return

        self.set_position(0, 0, altitude, 0)
        if self.current_state.mode != "OFFBOARD":
            if self.change_mode("OFFBOARD"):
                print("Offboard mode set successfully!")
            else:
                print("Failed to set Offboard mode.")
                return
    
    def arm_vehicle(self, arm: bool=True):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            arm_response = arm_service(arm)  # True to arm, False to disarm
            return arm_response.success
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return False
        
        
    def change_mode(self, mode: str):
        try:
            if mode == "OFFBOARD" and self.setpoint_thread is None:
                self.start_setpoint_thread()
                time.sleep(1)
            set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            response = set_mode_service(0, mode)
            return True
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return False
        
    #метод для передачи скорости    
    def set_position2(self, x: float, y: float, z: float, yaw: float=0):
        if self.current_state:
            self.change_mode("OFFBOARD")
        setpoint = PositionTarget()
        setpoint.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_YAW
        setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        setpoint.velocity.x = 0 # self.target_Vx
        setpoint.velocity.y = 0 #self.target_Vy
        setpoint.velocity.z = self.target_Vz
        setpoint.yaw_rate = self.target_yaw_rate
        print(f'целевая скорость по координате x {self.target_Vx}') 
        print(f'целевая скорость по координате y {self.target_Vy}') 
        print(f'целевая скорость по координате z {self.target_Vz}') 
        print(f'целевая скорость по yaw {self.target_yaw_rate}') 
        return setpoint
        
    #метод для передачи координаты *используется при взлете     
    def set_position(self, x: float, y: float, z: float, yaw: float=0):
        if self.current_state:
            self.change_mode("OFFBOARD")
        setpoint = PositionTarget()
        setpoint.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_YAW_RATE
        setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        setpoint.position.x = x
        setpoint.position.y = y
        setpoint.position.z = z
        setpoint.yaw = yaw
        self._setpoint = setpoint

   
if __name__ == "__main__":
    rospy.init_node('test_flight_commander')
    commander = FlightCommander()
    commander.do_takeoff()
    time.sleep(5)
   # commander.set_position(0, 0, 3, 0)
# while (self.current_state is None):
#            pass
    #if not self.current_state.armed:
    #    if self.arm_vehicle():
    #        print("Vehicle armed successfully!")
    #    else:
    #        print("Failed to arm the vehicle.")
    #cmd = self.pose_pub.publish(self.set_position(0, 0, 1, 0)
    #print(cmd)
    #print(self.current_state.mode)
#    if self.current_state.mode != "OFFBOARD":
#        if self.change_mode("OFFBOARD"):
#            print("Offboard mode set successfully!")
#        else:
#            print("Failed to set Offboard mode.")
            
    
    #count = 0
    #while count < 1000:
    #    print(count)
    #    self._setpoint = commander.set_position2(0, 0, 3, 0)
    #    count += 1
    #    time.sleep()
    #time.sleep(20)
    #commander.set_position(0, 0, 3, 0)
    #time.sleep(10)
    #commander.set_position(-5, -3, 3, -1.57)
    
    #time.sleep(10)
    #commander.set_position(-5, -11, 3, -1.57)
    #time.sleep(10)
    
    #time.sleep(10)
    #commander.set_position(-15, -11, 3, -1.57)
    #time.sleep(10)
    
    #commander.set_position(-10, 8, 3, 1.57)
    #time.sleep(10)
    
    #commander.set_position(-10, 23, 3, 1.57)
    #time.sleep(10)
    #commander.set_position(-5, 23, 3, 1.57)
    #time.sleep(10)
    
    #commander.set_position(5, 23, 3, 1.57)
    #time.sleep(10)
    
    #commander.set_position(3, 8, 3, 1.57)
    #time.sleep(10)
    #commander.set_position(-0, 0, 3, 1.57)
    #time.sleep(10)
    commander.land_vehicle()
