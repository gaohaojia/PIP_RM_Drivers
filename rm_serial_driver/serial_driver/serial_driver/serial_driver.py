import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist

from rm_interfaces.msg import Referee, VisionRecv, Target

import struct
import serial
from queue import Queue
import threading
import time

# 发送和接收频率（Hz）
TRANSMIT_RATE = 500
RECEIVE_RATE = 500

global_serial = None

# 初始化串口
def init_serial() -> serial.Serial:
    global global_serial
    cnt = 0
    while True:
        try:
            global_serial = serial.Serial(
                port=f"/dev/ttyACM{cnt}",
                baudrate=1000000
                # parity=serial.PARITY_EVEN
            )
            break
        except:
            cnt += 1
            if cnt > 10:
                cnt = 0
            continue
    print("打开串口")

# CRC计算
def crc16(data_pack: list[int]) -> list[bytes]:
    crc = 0xFFFF # 初始化CRC校验值为0xFFFF
    for byte in data_pack:
        crc ^= byte # 将当前数据字节与CRC校验值进行异或运算
        for _ in range(8):
            if crc & 1:
                crc >>= 1
                crc ^= 0xA001 # 使用CRC16多项式进行异或运算
            else:
                crc >>= 1
    return [bytes([byte]) for byte in struct.pack('<H', crc)]

class Receiver():
    def __init__(self, data_packet_queue: Queue):
        self.data_packet_queue = data_packet_queue

    def receive(self):
        while True:
            time.sleep(1.0 / RECEIVE_RATE)

            # 判断头字节
            try:
                if global_serial.read() != b'\x5A':
                    continue
            except:
                init_serial()
                continue
            
            # 获取其余所有数据
            try:
                data_packet = [bytes([byte]) for byte in global_serial.read(16)]
            except:
                init_serial()
                continue
            
            if self.data_packet_queue.full():
                self.data_packet_queue.get()
            self.data_packet_queue.put(data_packet)

# 串口发送器
class Transmitter():
    def __init__(self, nav_pack_queue: Queue, aim_pack_queue: Queue) -> None:
        self.nav_pack_queue: Queue = nav_pack_queue
        self.aim_pack_queue: Queue = aim_pack_queue

    def transmit(self):
        while True:
            time.sleep(1.0 / TRANSMIT_RATE)
            
            # 判断是否有导航数据
            if not self.nav_pack_queue.empty():
                nav_pack = self.nav_pack_queue.get()
                data_pack = [b'\xA4']
                data_pack.extend([bytes([byte]) for byte in nav_pack])
                data_pack.extend(crc16(nav_pack))
                try:
                    for data in data_pack:
                        global_serial.write(data)
                except:
                    init_serial()
                    continue

            # 判断是否有自瞄数据
            if not self.aim_pack_queue.empty():
                aim_pack = self.aim_pack_queue.get()
                data_pack = [b'\xA5']
                data_pack.extend([bytes([byte]) for byte in aim_pack])
                data_pack.extend(crc16(aim_pack))
                try:
                    for data in data_pack:
                        global_serial.write(data)
                except:
                    init_serial()
                    continue
            

# 串口通信节点
class Serial_driver(Node):

    def __init__(self, name):
        super().__init__(name)

        # 接收导航速度数据
        self.twist_sub = self.create_subscription(Twist, '/cmd_vel', self.vel_callback, 10)
        self.twist_sub = self.create_subscription(Target, '/tracker/target', self.target_callback, qos_profile_sensor_data)

        # 发送导航数据
        self.referee_pub = self.create_publisher(Referee, "referee_data", 10)
        self.vision_pub = self.create_publisher(VisionRecv, "vision_recv", 10)

        # 多进程实现串口同时读写
        self.nav_pack_queue = Queue(maxsize=3)
        self.aim_pack_queue = Queue(maxsize=3)
        self.data_packet_queue = Queue(maxsize=3)
        
        # 初始化串口
        init_serial()
        
        # 开启串口多线程
        self.transmitter = Transmitter(self.nav_pack_queue, self.aim_pack_queue)
        self.receiver = Receiver(self.data_packet_queue)
        threads = [threading.Thread(target=self.transmitter.transmit),
                   threading.Thread(target=self.receiver.receive)]
        [thread.start() for thread in threads]

        # 发布计时器
        self.pub_timer = self.create_timer(0.001, self.publisher_callback)

    # 导航数据接收回调
    def vel_callback(self, msg: Twist):
        self.get_logger().info(f"X:{msg.linear.x}, Y:{msg.linear.y}, Z:{msg.linear.z}, a:{msg.angular.x}, b:{msg.angular.y}, c:{msg.angular.z}")

        data_pack = [i for i in struct.pack('<i', int(msg.linear.x * 100000))]
        data_pack.extend([i for i in struct.pack('<i', int(msg.linear.y * 100000))])
        data_pack.extend([i for i in struct.pack('<i', int(msg.linear.z * 100000))])
        data_pack.extend([i for i in struct.pack('<i', int(msg.angular.x * 100000))])
        data_pack.extend([i for i in struct.pack('<i', int(msg.angular.y * 100000))])
        data_pack.extend([i for i in struct.pack('<i', int(msg.angular.z * 100000))])
        data_pack.extend([0 for i in range(23)])
        
        if self.nav_pack_queue.full():
            self.nav_pack_queue.get()
        self.nav_pack_queue.put(data_pack)
    
    # 视觉数据回调
    def target_callback(self, msg: Target):

        map = {"":0, "outpost":0, "1":1, "2":2, "3":3, "4":4, "5":5, "guard":6, "base":7}
        # self.get_logger().info(f"Get Target: {msg.position.x}")

        data_pack = [1 if msg.tracking else 0]
        data_pack.append(map[msg.id])
        data_pack.append(msg.armors_num)
        data_pack.extend([i for i in struct.pack('<i', int(msg.position.x*100000))])
        data_pack.extend([i for i in struct.pack('<i', int(msg.position.y*100000))])
        data_pack.extend([i for i in struct.pack('<i', int(msg.position.z*100000))])
        data_pack.extend([i for i in struct.pack('<i', int(msg.yaw*100000))])
        data_pack.extend([i for i in struct.pack('<i', int(msg.velocity.x*100000))])
        data_pack.extend([i for i in struct.pack('<i', int(msg.velocity.y*100000))])
        data_pack.extend([i for i in struct.pack('<i', int(msg.velocity.z*100000))])
        data_pack.extend([i for i in struct.pack('<i', int(msg.v_yaw*100000))])
        data_pack.extend([i for i in struct.pack('<i', int(msg.radius_1*100000))])
        data_pack.extend([i for i in struct.pack('<i', int(msg.radius_2*100000))])
        data_pack.extend([i for i in struct.pack('<i', int(msg.dz*100000))])
        
        if self.aim_pack_queue.full():
            self.aim_pack_queue.get()
        self.aim_pack_queue.put(data_pack)
    
    # 发布计时器回调
    def publisher_callback(self):
        if not self.data_packet_queue.empty():
            data_pack = self.data_packet_queue.get()

            vmsg = VisionRecv()
            vmsg.detect_color = data_pack[0][0]
            vmsg.reset_tracker = bool(data_pack[1][0])
            vmsg.roll = struct.unpack('<h', b''.join(data_pack[2:4]))[0] / 100.0
            vmsg.pitch = struct.unpack('<h', b''.join(data_pack[4:6]))[0] / 100.0
            vmsg.yaw = struct.unpack('<h', b''.join(data_pack[6:8]))[0] / 100.0
            # vmsg.aim_x = struct.unpack('<i', b''.join(data_pack[8:12]))[0] / 100000.0
            # vmsg.aim_y = struct.unpack('<i', b''.join(data_pack[12:16]))[0] / 100000.0
            # vmsg.aim_z = struct.unpack('<i', b''.join(data_pack[16:20]))[0] / 100000.0
            self.vision_pub.publish(vmsg)
            
            nmsg = Referee()
            nmsg.sentry_hp = struct.unpack('<h', b''.join(data_pack[8:10]))[0]
            nmsg.base_hp = struct.unpack('<h', b''.join(data_pack[10:12]))[0]
            nmsg.ammo = struct.unpack('<h', b''.join(data_pack[12:14]))[0]
            nmsg.remaining_time = struct.unpack('<h', b''.join(data_pack[14:16]))[0]
            self.referee_pub.publish(nmsg)

def main(args=None):
    rclpy.init(args=args)
    node = Serial_driver("serial_driver")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

