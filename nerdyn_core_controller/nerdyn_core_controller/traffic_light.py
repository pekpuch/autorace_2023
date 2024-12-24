import os
import numpy as np
import copy
from enum import Enum
from geometry_msgs.msg import Twist
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import String


class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')
        self.final = self.create_publisher(String,'/robot_finish',10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscribtion = self.create_subscription(Image, '/color/image', self.detect_traffic_light_color_callback, 10)
        self.pose_subscriber = self.create_subscription(Image, '/depth/image', self.update_pose, 10)
        self.time_start =time.time()
        self.time_end = 0
        self.sign=None
        self._bridge = CvBridge()
        self.is_green_light = False
        self.light_is_done = False
        self.mode = True
        self.stage='START'
        self.scan = Image()
        self.aloha = False
        self.get_logger().info(str(os.getcwd()))
        self.was_block = False
    def update_pose(self, data): # камера глубины
        cv_image = self._bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        depth_array = np.array(cv_image, dtype=np.float32).clip(min=0, max=0.5)
        depth_image = cv2.normalize(depth_array, None, 0, 255, cv2.NORM_MINMAX)
        depth_image = np.uint8(depth_image)

        if (self.stage == "CROSS" or self.stage == "CROSS2"): # на этапе перекерстка нужно объехать кртвую коллизию знака
            if np.mean(depth_image[:250,590:750] < 200):
                uwu = Twist()
                #self.get_logger().info("ЛЕВА РУЛЯЯЯЯ")
                uwu.angular.z = 0.25
                self.publisher.publish(uwu)
                time.sleep(0.07)
            else :
                uwu = Twist()
                uwu.angular.z = 0.0
                self.publisher.publish(uwu)
        height, width = depth_image.shape
        if (self.stage == "SIGNDETECT"): # на этапе детекции знака мы используем камеру глубины для классификации знака
            height, width = depth_image.shape
            time.sleep(0.5)
            left_half = depth_image[:height//2, :width//2-150]
            right_half = depth_image[:height//2, width//2-150:]
            left_mean = np.mean(left_half)
            right_mean = np.mean(right_half)
            self.get_logger().info(str(left_mean))
            self.get_logger().info(str(right_mean))
            if right_mean > left_mean:
                self.stage = "LEFTSIGN"
            else:
                self.stage = "RIGHTSIGN"

        if (self.stage == "TURNLEFT3" or self.stage == "TURNRIGHT1"): # во время езды по линии мы проверяем наличие строительных блоков
            if np.mean(depth_image[:5,width//2-5:width//2+5] < 80):
                self.was_block = True

                if self.stage == "TURNLEFT3":
                    self.stage = "BLOCKSL"
                else:
                    self.stage="BLOCKSR"

        if self.stage == "BLOCKSR" or self.stage=='BLOCKSL': # проверка наличия блоков
            self.block = (np.mean(depth_image[:5,width//2-5:width//2+5] < 80)) # перед нами блок
            self.get_logger().info(str(self.block))

    def detect_traffic_light_color_callback(self, image_msg):  
        if self.mode == True:
            frame = self._bridge.imgmsg_to_cv2(image_msg, "bgr8")
            new_frame = frame[300:320]
            height,width,channels = frame.shape
            target_color_green = np.array([2, 110, 2])
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame_gray1 = frame_gray / 255.0
            tres = frame_gray1 > 0.5
            wb_frame = tres.astype(np.uint8)*255
            # cv2.imshow("camera",frame[100:height//3+100,2*width//4:3*width//4])
            # cv2.waitKey(1)
            self.get_logger().info(str(self.stage))

            wb_frame = wb_frame[:,(width//5) +30:(4*width//5)-50]
            if self.stage=="TURNRIGHT1" and self.was_block:
                lb = np.array([110,50,50])
                ub = np.array([130,255,255])
                mask = cv2.inRange(frame[100:height//3+100,2*width//4:3*width//4],lb,ub)
                blue = np.count_nonzero(mask)
                if blue>900:
                    stop = Twist()
                    stop.angular.z = 0.0
                    stop.linear.x = 0.0
                    self.publisher.publish(stop)
                    time.sleep(0.1)
                    
                    
                    self.time_end = time.time()
                    self.stage = "ITSOVER"

                    msg = String()
                    msg.data = f"Nerdy Nest:"
                    self.final.publish(msg)
            if self.stage == "ITSOVER":
                stop = Twist()
                stop.angular.z = 0.0
                stop.linear.x = 0.0
                self.publisher.publish(stop)

                # self.get_logger().info(f"колво голубых {blue}")
            go = Twist()
            mask = np.all(frame == target_color_green, axis=-1) # фиксируем зеленый свет камерой
            if self.stage == 'START':                           # этап старт от зеленого до поворота
                if np.any(mask == True):
                    self.is_green_light = True
                    
                if (np.mean(frame_gray[:,490:530]) < 168) and self.is_green_light:
                    go.linear.x = 0.1
                    go.angular.z = 0.0
                    self.publisher.publish(go)
                    time.sleep(0.1)
                else:
                    go.linear.x=0.0
                    self.publisher.publish(go)
                    if self.is_green_light:
                        self.stage = 'TURNLEFT' # Переход к повороту налево
            
            if self.stage=='TURNLEFT': # Поворачиваем налево
                if (np.mean(frame_gray[:,490:530])) > 160:
                    go = Twist()
                    go.angular.z = 0.42
                    go.linear.x = 0.1
                    self.publisher.publish(go)
                else: self.stage = 'CROSS' # Переходим к этапу перекресток
            
            if self.stage=='CROSS': # едем прямо до следующегот поворота
                go = Twist()
                if (np.mean(frame_gray[:,490:530]) < 165):
                    go.linear.x = 0.1
                    self.publisher.publish(go)
                    time.sleep(0.1)

                else:
                    go.angular.z=0.0
                    go.linear.x = 0.0
                    self.publisher.publish(go)
                    self.stage="CROSS2" 

            if self.stage=="CROSS2": # этап для выравнивания 
                if (np.mean(frame_gray[:,490:530]) < 180):
                    go.linear.x = 0.1
                    self.publisher.publish(go)
                    time.sleep(0.1)
                else:
                    go.linear.x=0.0
                    self.publisher.publish(go)
                    if self.is_green_light:
                        self.stage = 'TURNLEFT2' # Переходим ко втормоу повороту перед знаками

            if self.stage=='TURNLEFT2':  # ВЫполняем второй поворот
                if (np.mean(frame_gray[:,490:530])) > 160:
                    go = Twist()
                    go.angular.z = 0.6
                    go.linear.x = 0.05
                    self.publisher.publish(go)
                else: 
                    
                    self.stage = 'CROSS3' # этап Выравнивания перед детекцией

            if self.stage=="CROSS3": # Выравниваемся и останавливаемся для детекции
                go = Twist()
                go.angular.z = 0.0
                go.linear.x = 0.1
                if (np.mean(frame_gray[:,490:530])) > 125:
                    go.angular.z = 0.4
                self.publisher.publish(go)
                time.sleep(0.01)
                if (np.mean(frame_gray[:,490:530])) < 116:
                    go.angular.z = 0.45
                    go.linear.x = 0.8
                    self.publisher.publish(go)
                    time.sleep(0.2)
                    go.linear.x = 0.3
                    self.publisher.publish(go)
                    time.sleep(0.5)
                    go.angular.z = 0.0
                    go.linear.x = 0.0
                    self.publisher.publish(go)
                    time.sleep(0.5)
                    self.stage = "SIGNDETECT" # Переходим к детекции

            if self.stage == "SIGNDETECT": # останавливаемся и переходим в камеру глубины
                go = Twist()
                go.angular.z = 0.0
                go.linear.x = 0.0
                self.publisher.publish(go)
                

            if self.stage == "LEFTSIGN":  # левый знак
                self.sign = -1
                go.angular.z = 0.9
                self.publisher.publish(go)
                time.sleep(1.7)
                self.stage = "TURNLEFT3" # идем по левой ллинии

            if self.stage == "RIGHTSIGN": # Правый знак
                self.sign=1
                go.angular.z = -0.9
                self.publisher.publish(go)
                time.sleep(1.7)
                self.stage = "TURNRIGHT1" # Идем по правой линии

            if self.stage=="TURNRIGHT1" or self.stage=="TURNLEFT3": # Указываем по какой линии ехать
                go = Twist()
                go.angular.z = 0.0
                go.linear.x = 0.0
                self.publisher.publish(go)
                visor = frame_gray[6*(height//8):,:]
                if self.stage == "TURNLEFT3":
                    self.go_forward(wb_frame[-5],"left") # запускаем метод для езды
                if self.stage == "TURNRIGHT1":
                    self.go_forward(wb_frame[-5],"right")

            # if self.stage=="BLOCKSL":
            #     go = Twist()

                
            #     # право вперед
            #     go.angular.z = -0.9
            #     go.linear.x = 0.0
            #     self.publisher.publish(go)
            #     time.sleep(0.5)
            #     go.angular.z = 0.0
            #     go.linear.x = 0.2
            #     self.publisher.publish(go)
            #     time.sleep(0.5)
                
            #     go.angular.z = 0.9
            #     go.linear.x = 0.0
            #     self.publisher.publish(go)
            #     time.sleep(0.5)
            #     go.angular.z = 0.0
            #     go.linear.x = 0.2
            #     self.publisher.publish(go)
            #     time.sleep(0.5)

            #     go.angular.z = 0.9
            #     go.linear.x = 0.0
            #     self.publisher.publish(go)
            #     time.sleep(0.5)
            #     go.angular.z = 0.0
            #     go.linear.x = 0.2
            #     self.publisher.publish(go)
            #     time.sleep(0.5)

            #     go.angular.z = -0.9
            #     go.linear.x = 0.0
            #     self.publisher.publish(go)
            #     time.sleep(0.5)
            #     go.angular.z = 0.0
            #     go.linear.x = 0.2
            #     self.publisher.publish(go)
            #     time.sleep(0.5)

            #     go.angular.z = -0.9
            #     go.linear.x = 0.0
            #     self.publisher.publish(go)
            #     time.sleep(0.5)
            #     go.angular.z = 0.0
            #     go.linear.x = 0.2
            #     self.publisher.publish(go)
            #     time.sleep(0.5)

            #     go.angular.z = 0.9
            #     go.linear.x = 0.0
            #     self.publisher.publish(go)
            #     time.sleep(0.5)
            #     go.angular.z = 0.0
            #     go.linear.x = 0.2
            #     self.publisher.publish(go)
            #     time.sleep(0.5)
            #     self.stage = "TURNRIGHT1"


            # if self.stage=="BLOCKSR":
            #     go = Twist()

            #     go.angular.z = 0.9
            #     go.linear.x = 0.0
            #     self.publisher.publish(go)
            #     time.sleep(0.5)
            #     go.angular.z = 0.0
            #     go.linear.x = 0.2
            #     self.publisher.publish(go)
            #     time.sleep(0.5)

            #     go.angular.z = -0.9
            #     go.linear.x = 0.0
            #     self.publisher.publish(go)
            #     time.sleep(0.5)
            #     go.angular.z = 0.0
            #     go.linear.x = 0.2
            #     self.publisher.publish(go)
            #     time.sleep(0.5)

            #     go.angular.z = -0.9
            #     go.linear.x = 0.0
            #     self.publisher.publish(go)
            #     time.sleep(0.5)
            #     go.angular.z = 0.0
            #     go.linear.x = 0.2
            #     self.publisher.publish(go)
            #     time.sleep(0.5)

            #     go.angular.z = 0.9
            #     go.linear.x = 0.0
            #     self.publisher.publish(go)
            #     time.sleep(0.5)
            #     go.angular.z = 0.0
            #     go.linear.x = 0.2
            #     self.publisher.publish(go)
            #     time.sleep(0.5)


            #     self.stage="TURNRIGHT1"







            if self.stage=="BLOCKSL" : # если видим блок, прерываем движение, запускаем метод для обхезда блока
                self.go_forward_blocks(wb_frame[5*height//6],'left')
            if self.stage == "BLOCKSR":
                self.go_forward_blocks(wb_frame[5*height//6],'right')
            if self.stage=="its_OVER" :
                self.its_OVER()
    
    def go_forward(self,visor,follow_side = None):
        visor = visor/255.0
        left_side = visor[0:len(visor)//2]
        right_side = visor[len(visor)//2:]
        go = Twist()
        if follow_side == 'left':  # едем по левой линии
            if np.sum(left_side)==10 and np.sum(right_side)==0: # если линии нет
                go.angular.z=0.0
                go.linear.x=0.6
                self.publisher.publish(go)
                time.sleep(0.05)
                return 0 
            else:
                if np.sum(visor) == len(visor):   # если над пропастью
                    go.angular.z=0.3
                    go.linear.x = 0.0
                    self.publisher.publish(go)
                    time.sleep(0.05)
                    return 1
                if np.sum(left_side) > np.sum(right_side) and np.sum(left_side)>10:  # стремимся к тому чтобы в левой части визора было несколько левых пикселей
                    go.angular.z=-0.2
                    go.linear.x = 0.05
                    self.publisher.publish(go)
                    time.sleep(0.05)
                    return 0
                if (np.sum(left_side) < 10 and np.sum(left_side) < np.sum(right_side)) or np.sum(left_side)<10:
                    go.angular.z = 0.2
                    go.linear.x = 0.05
                    self.publisher.publish(go)
                    time.sleep(0.05)
                    return 0
        if follow_side == 'right': # то же самое но для правой линии
            if np.sum(left_side)==0 and np.sum(right_side)==10:
                go.angular.z=0.0
                go.linear.x=0.6
                self.publisher.publish(go)
                time.sleep(0.05)
                return 0 
            else:
                if np.sum(visor) == len(visor):
                    go.angular.z=0.3
                    go.linear.x = 0.0
                    self.publisher.publish(go)
                    time.sleep(0.05)
                    return 1
                if(np.sum(left_side) > np.sum(right_side) and np.sum(right_side)<10) or np.sum(left_side)<10 and  np.sum(right_side)<10:
                    go.angular.z=-0.2
                    go.linear.x = 0.05
                    self.publisher.publish(go)
                    time.sleep(0.05)
                    return 0
                if np.sum(right_side) > 10 and np.sum(left_side) < np.sum(right_side):
                    go.angular.z = 0.2
                    go.linear.x = 0.05
                    self.publisher.publish(go)
                    time.sleep(0.05)
                    return 0

    def go_forward_blocks(self,visor,follow_side):
        visor = visor/255.0
        center_visor = visor[len(visor)//3:2*len(visor)//3]
        go = Twist()
        
        if follow_side == 'left': # тут разница только в кожффицентах для разворота вправо и влево
            
            if self.block: # видим блок
                go.angular.z = self.sign * 0.55 # поворачиваем пока блок не пропадет из поля зрения
                go.linear.x = 0.0
                self.publisher.publish(go)
                time.sleep(0.5)
            else:
                go.angular.z=0.0
                self.publisher.publish(go)
                time.sleep(0.1)
            if np.sum(center_visor) != len(center_visor) and not self.block: # если мы не на линии то едем вперед
                go.linear.x=0.1
                go.angular.z=0.0
                self.publisher.publish(go)
                time.sleep(0.1)
            elif np.sum(center_visor) == len(center_visor) and not self.block: # если мы на линии то запускаем езду по ней предварительно повернувшись
                go.linear.x=0.0
                self.publisher.publish(go)
                time.sleep(0.1)
                go.angular.z = -self.sign * 1.0
                self.publisher.publish(go)
                time.sleep(0.9)
                
                if self.sign == -1:
                    line_dir = "TURNRIGHT1" 
                else:
                    line_dir = "TURNLEFT3" 
                self.sign = -self.sign
                self.stage = line_dir

        if follow_side == 'right': #  то же самое
            if self.block:
                go.angular.z = self.sign * 0.5
                go.linear.x = 0.0
                self.publisher.publish(go)
                time.sleep(0.5)
            else:
                go.angular.z=0.0
                self.publisher.publish(go)
                time.sleep(0.1)
            
            if np.sum(center_visor) != len(center_visor) and not self.block:
                go.linear.x=0.1
                go.angular.z=0.0
                self.publisher.publish(go)
                time.sleep(0.1)
            
            elif np.sum(center_visor) == len(center_visor) and not self.block:
                go.linear.x=0.0
                self.publisher.publish(go)
                time.sleep(0.1)
                go.angular.z = -self.sign * 1.0
                self.publisher.publish(go)
                time.sleep(0.9)
                
                if self.sign == -1:
                    line_dir = "TURNRIGHT1" 
                else:
                    line_dir = "TURNLEFT3" 
                self.sign = -self.sign
                self.stage = line_dir

    # def its_OVER(self):
        
#                                        ,//(##%&@@@@%,                               
#                          *@@@@@@@@@@@@@@@@@@@@@@@@@@@/.                         
#                       /@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@&/.                   
#                      *@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@&*                
#                     .&@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@/             
#                    (@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@/         
#                   #@@@@@@@@@@@@@@@@#,         ./%@@@@@@@@@@@@@@@@@@@@@@@%.      
#                  &@@@@@@@@@@@@@@.                   .(%%##(%@@@@@@@@@@@@@@%     
#                *&(@@@@@@@@@@@@#                               .#&&&@@@@@@@@@%.  
#               (* *@@@@@@@@@@/                                      ,&@@@@@@@@@# 
#             *#,  /@@@@@@@@/                                            ,(,(@@@% 
#            (#    *@@@@@&.                                                 .&@@&.
#           %/                                                               .&#  
#         .#*                                                        .        #*  
#        .#,                                                         ,.      ,&,  
#        #*                      %@@@@#.          ,@@/               .,      (#   
#       ((                     .&%    .(@&*        #@@@&*            *,     /#.   
#      *#                       *%        *%@&(,#@%(,,%@@@#         .*     (%.    
#     .#                                      ,@%   .*%@@@@@@,            (%      
#    .(,                                     ,@/         ,&@@@@/  .,.    ,&.      
#   .%.                                     (@,   ,.        .#@@&.       &/       
#  .#                                      (@/   .#@@@&&#*.    .@@      &#        
#  %.                                        (@@,     */*(%@(  .@@@@@#*&@&.       
# ,                                             /&@(*, ./.    *@#     #@@,        
#          ,                                        ,(@&#*.  #@,      %@,         
#         .*                                            ,&*/&(        %*          
#        .*                                           .#*             @*          
#      ,/.                                           ((.             .@,          
#                                                  *@/.              *%           
#                                               .#%%/               ,&.           
# *&&.                                     /#,     .%&..(%#*        (/            
#                                        .(.             (#/      .(/             
# *#                                     (,            *@.,/#%@@&(.               
# ,@@/                                      ,@@@@@&(.  #/                         
# ,@@@&,                                          ./%@@#                          
# ,@@@@@&,                               .(%//(%@@&%#*                            
# ,@@@@@@@&.         .@&%/*&@@#@@@@@%%##/.                                        
# ,@@@@@@@@@@.                 @*                                                 
# ,@@@@@@@@@@@%**,*,.         ,@,                                                 
# ,@@@@@@@@@@@@@@*            ,@,                                                 

def main():
    rclpy.init()
    node = TrafficLightDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
