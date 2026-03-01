#!/usr/bin/python3
# -*- coding: utf-8 -*-
import math
import os
import tkinter as tk
from tkinter import messagebox, simpledialog
import rclpy
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from msg_process.msg import ReceiveData as receive_data
from msg_process.msg import DecisonSendData as send_data
from msg_process.msg import AutoaimToDecision as autoaim_to_decision
from std_msgs.msg import Float64MultiArray
import threading


def publisher(node: Node, pub, x: float, y: float):
    pursuit_point = PointStamped()
    pursuit_point.point.x = x
    pursuit_point.point.y = y
    pursuit_point.header.frame_id = "map"
    pursuit_point.header.stamp = node.get_clock().now().to_msg()
    pub.publish(pursuit_point)


def to_RMUC(x, y):
    return x / 1800 * 28, 15 - y / 966 * 15


def to_RMUC_str(x, y):
    return f"{x / 1800 * 28:.3f}, {15 - y / 966 * 15:.3f}"

def from_RMUC(x, y):
    return int(x * 1800 / 28), int((15 - y) * 966 / 15)


class ImageAnnotator:
    def __init__(self, root, image_path, node: Node, pub):
        self.root = root
        self.root.title("RMUC地图标点发布/pursuit_point")
        self.node = node
        self.pub = pub

        # 检查图片文件是否存在，如果不存在则创建一个简单的画布
        if os.path.exists(image_path):
            try:
                # tkinter.PhotoImage 支持 PNG, GIF, PPM/PGM 等格式
                self.image = tk.PhotoImage(file=image_path)
                canvas_width = self.image.width()
                canvas_height = self.image.height()
                print(f"Successfully loaded image: {image_path}")
            except tk.TclError as e:
                print(f"Error loading image {image_path}: {e}")
                # 如果图片格式不支持，创建一个默认尺寸的画布
                canvas_width = 1800
                canvas_height = 966
                self.image = None
        else:
            # 如果图片不存在，创建一个默认尺寸的画布
            canvas_width = 1800
            canvas_height = 966
            self.image = None
            print(f"Warning: Image file {image_path} not found. Using default canvas size.")

        self.canvas = tk.Canvas(root, width=canvas_width, height=canvas_height, bg='white')
        self.canvas.pack()

        if self.image:
            self.canvas.create_image(0, 0, anchor='nw', image=self.image)
        else:
            # 如果没有图片，绘制一个简单的网格作为参考
            for i in range(0, canvas_width, 100):
                self.canvas.create_line(i, 0, i, canvas_height, fill='lightgray')
            for i in range(0, canvas_height, 100):
                self.canvas.create_line(0, i, canvas_width, i, fill='lightgray')

        
        self.is_distance = tk.IntVar(value=1)
        self.is_line = tk.IntVar(value=1)
        self.is_forward = tk.IntVar(value=1)
        self.is_goal_point = tk.IntVar(value=1)
        self.is_autoaim = tk.IntVar(value=1)
        self.is_radar = tk.IntVar(value=1)
        self.is_time = tk.IntVar(value=1)

        self.check1 = tk.Checkbutton(root, text="Target Distance ", font=("Ubuntu Mono", 15), background="yellow",
                                     variable=self.is_distance)
        self.check1.pack(side=tk.LEFT)

        self.check2 = tk.Checkbutton(root, text="Target Line ", font=("Ubuntu Mono", 15), background="purple",
                                     variable=self.is_line)
        self.check2.pack(side=tk.LEFT)

        self.base_spin = False
        self.check3 = tk.Checkbutton(root, text="Current Forward ", font=("Ubuntu Mono", 15),
                                     background="cyan" if self.base_spin else "limegreen", variable=self.is_forward)
        self.check3.pack(side=tk.LEFT)

        self.check4 = tk.Checkbutton(root, text="Goal Point ", font=("Ubuntu Mono", 15),
                                     background="orange" if self.is_goal_point.get() else "",
                                     variable=self.is_goal_point, command=self.on_check_goal_point)
        self.check4.pack(side=tk.LEFT)

        self.check5 = tk.Checkbutton(root, text="AutoaimToDecision Log ", font=("Ubuntu Mono", 15),
                                     background="red" if self.is_autoaim.get() else "",
                                     variable=self.is_autoaim)
        self.check5.pack(side=tk.LEFT)

        self.check6 = tk.Checkbutton(root, text="Radar Point ", font=("Ubuntu Mono", 15),
                                     background="blue" if self.is_radar.get() else "",
                                     variable=self.is_radar)
        self.check6.pack(side=tk.LEFT)

        self.check7 = tk.Checkbutton(root, text="Time ", font=("Ubuntu Mono", 15),
                                     background="cyan" if self.is_time.get() else "",
                                     variable=self.is_time)
        self.check7.pack(side=tk.LEFT)

        self.hp = 400
        self.enemy_hp = [0, 0, 0, 0, 0, 0]
        self.enemy_outpost_hp = 1500
        self.enemy_base_hp = 5000
        self.cur_pose: list = []
        self.pursuit_point = None
        self.distance_show = None
        self.pursuit_line = None
        self.goal_point: list = []
        self.autoaim_target: list = []
        self.autoaim_disappear: list = []
        self.radar_point: list = []
        self.time = None

        self.rmuc_cur_x: float = 0
        self.rmuc_cur_y: float = 0
        self.cur_yaw = .0
        # 点在画布上的坐标，用于画连线
        self.cur_x = .0
        self.cur_y = .0
        self.pursuit_x = .0
        self.pursuit_y = .0

        # 巡逻区标志位
        self.is_in_patrol_area = False

        self.canvas.bind("<Button-1>", self.add_pursuit_point)

    def add_pursuit_point(self, event):
        if self.pursuit_point:
            # 删除上一个标点
            self.canvas.delete(self.pursuit_point[0])
            self.canvas.delete(self.pursuit_point[1])

        #获取鼠标点击时的x,y坐标
        x, y = event.x, event.y
        self.pursuit_x, self.pursuit_y = x, y
        #椭圆的外接矩形，这里就是圆形 fill内部填充颜色 outline边框颜色
        circle = self.canvas.create_oval(x - 9, y - 9, x + 9, y + 9, fill="yellow", outline="yellow")

        # Create a label to show the coordinates
        label = self.canvas.create_text(x + (80 if x < 80 else -85 if x > 1800 - 80 else 0),
                                        y + (18 if y < 966 / 2 else -35), text=to_RMUC_str(x, y),
                                        font=("Ubuntu Mono", 16), anchor='n')

        # Bind a click event to the label to update its position
        # self.canvas.tag_bind(label, "<Button-1>", lambda e, l=label, c=circle: self.on_label_click(e, l, c))

        self.pursuit_point = (circle, label)
        #  *：解包，将x,y分成独立的参数传入publisher函数
        publisher(self.node, self.pub, *to_RMUC(x, y))

        self.draw_line()

    def refresh_cur_point(self, msg: Odometry):
        if self.cur_pose:
            for item in self.cur_pose:
                # 删除上一个标点相关项目
                self.canvas.delete(item)
            self.cur_pose.clear()

        # 画当前点
        rmuc_x, rmuc_y = msg.pose.pose.position.x + 4.971, msg.pose.pose.position.y + 7.5
        self.rmuc_cur_x, self.rmuc_cur_y = rmuc_x, rmuc_y
        x, y = from_RMUC(rmuc_x, rmuc_y)
        self.cur_x, self.cur_y = x, y

        cur_pos = None
        if not self.is_in_patrol_area:
            cur_pos = self.canvas.create_oval(x - 9, y - 9, x + 9, y + 9, fill="cyan" if self.base_spin else "limegreen", outline="cyan" if self.base_spin else "limegreen")
        # 巡逻区识别时用矩形显示当前点
        else:
            cur_pos = self.canvas.create_rectangle(x - 9, y - 9, x + 9, y + 9, fill="cyan" if self.base_spin else "limegreen", outline="cyan" if self.base_spin else "limegreen")

        # Create a label to show the coordinates
        label = self.canvas.create_text(x + (80 if x < 80 else -85 if x > 1800 - 80 else 0),
                                        y + (18 if y < 966 / 2 else -35), text=f"hp:{self.hp}\n{rmuc_x:.3f}, {rmuc_y:.3f}",
                                        font=("Ubuntu Mono", 16), anchor='n')
        self.cur_pose.extend([cur_pos, label])

        # 以当前点为起点，self.cur_yaw为指向角度画虚线直到画布边界
        if self.is_forward.get():
            orientation = msg.pose.pose.orientation
            tf2_quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
            _, _, self.cur_yaw = euler_from_quaternion(tf2_quaternion)
            arrow_x = x + 2000 * math.cos(self.cur_yaw)
            arrow_y = y - 2000 * math.sin(self.cur_yaw)
            forward = self.canvas.create_line(x, y, arrow_x, arrow_y, fill="cyan" if self.base_spin else "limegreen",
                                              width=1, dash=(10, 5))
            self.cur_pose.append(forward)

        self.draw_line()
        self.show_distance()

    def on_label_click(self, event, label, circle):
        # Get the current coordinates from the label text
        x, y = map(int, self.canvas.itemcget(label, "text")[1:-1].split(','))

        # Prompt the user for new coordinates
        new_coords = simpledialog.askstring("Update Coordinates", "Enter new coordinates (x,y):",
                                            parent=self.root, initialvalue=to_RMUC_str(x, y))

        if new_coords:
            print(new_coords)
            new_x, new_y = map(int, new_coords.split(','))

            # Update the circle and label positions
            self.canvas.coords(circle, new_x - 5, new_y - 5, new_x + 5, new_y + 5)
            self.canvas.coords(label, new_x, new_y - 15)
            self.canvas.itemconfig(label, text=to_RMUC(new_x, new_y))

    def show_distance(self):
        if self.distance_show:
            self.canvas.delete(self.distance_show)
        if not self.is_distance.get():
            return

        rmuc_cur_x, rmuc_cur_y = to_RMUC(self.cur_x, self.cur_y)
        rmuc_pursuit_x, rmuc_pursuit_y = to_RMUC(self.pursuit_x, self.pursuit_y)
        # 计算线段长度
        length = math.sqrt((rmuc_cur_x - rmuc_pursuit_x) ** 2 + (rmuc_cur_y - rmuc_pursuit_y) ** 2)
        text = self.canvas.create_text(35, 80, text=f"目标距离\n {length:.3f}m", font=("Ubuntu Mono", 20),
                                       fill="yellow", anchor="nw")

        self.distance_show = text

    def draw_line(self):
        if self.pursuit_line:
            # 删除上一条连线
            self.canvas.delete(self.pursuit_line[0])
            self.canvas.delete(self.pursuit_line[1])

        if not self.is_line.get():
            return

        # 画线
        line = self.canvas.create_line(self.cur_x, self.cur_y, self.pursuit_x, self.pursuit_y, fill="purple", width=3)
        rmuc_cur_x, rmuc_cur_y = to_RMUC(self.cur_x, self.cur_y)
        rmuc_pursuit_x, rmuc_pursuit_y = to_RMUC(self.pursuit_x, self.pursuit_y)
        # 计算线段长度
        length = math.sqrt((rmuc_cur_x - rmuc_pursuit_x) ** 2 + (rmuc_cur_y - rmuc_pursuit_y) ** 2)

        # 计算线段中点坐标
        mid_x = (self.cur_x + self.pursuit_x) / 2
        mid_y = (self.cur_y + self.pursuit_y) / 2
        offset = 10  # 偏移量
        angle = math.degrees(math.atan2(self.pursuit_y - self.cur_y, self.pursuit_x - self.cur_x))
        text_x = mid_x + offset * math.cos(math.radians(angle + 90))
        text_y = mid_y + offset * math.sin(math.radians(angle + 90))

        text = self.canvas.create_text(text_x, text_y, text=f"{length:.3f}m", font=("Ubuntu Mono", 16), anchor="center")

        self.pursuit_line = (line, text)

    def receive_data_callback(self, msg: receive_data):
        self.is_in_patrol_area = msg.is_in_patrol_area
        self.hp = msg.hp_sentry
        self.enemy_hp = [msg.enemy_hero_hp, msg.enemy_engineer_hp, msg.enemy_foot_3_hp, msg.enemy_foot_4_hp, msg.enemy_foot_5_hp, msg.enemy_sentry_hp]
        self.enemy_outpost_hp = msg.hp_enemy_outpost
        self.enemy_base_hp = msg.hp_enemy_base
        print(msg.is_in_patrol_area)
        print(msg.radar_data)
        if self.radar_point:
            for item in self.radar_point:
                self.canvas.delete(item)
            self.radar_point.clear()
        if self.time:
            self.canvas.delete(self.time)
            self.time = None
        if self.is_time.get():
            self.time = self.canvas.create_text(50, 35, text=f"{msg.time//60}:{msg.time%60:0>2}", font=("Ubuntu Mono", 30), fill="cyan", anchor="nw")
        if not self.is_radar.get():
            return
        if len(msg.radar_data) != 12:
            print("雷达数据长度非法！")
            return
        # 使用列表推导式分组
        radar_datas = [msg.radar_data[i:i + 2] for i in range(0, 12, 2)]
        for (idx, (rmuc_x, rmuc_y)) in enumerate(radar_datas):
            rmuc_y = 15 - rmuc_y
            if rmuc_x < .0 or rmuc_y < 0:
                continue
            x, y = from_RMUC(rmuc_x, rmuc_y)
            hp = self.enemy_hp[idx]
            circle = self.canvas.create_oval(x - 5, y - 5, x + 5, y + 5, fill="blue", outline="blue")
            label = self.canvas.create_text(x + (80 if x < 80 else -85 if x > 1800 - 80 else 0),
                                            y + (18 if y < 966 / 2 else -35), text=f"{idx+1} hp:{hp}\n{rmuc_x:.2f}, {rmuc_y:.2f}",
                                            font=("Ubuntu Mono", 14), anchor='n')
            self.radar_point.extend([circle, label])

    def send_data_callback(self, msg: send_data):
        self.base_spin = (msg.spin_mode == 1)

    def on_check_goal_point(self):
        if self.is_goal_point.get() == 0:
            if not self.is_goal_point.get():
                for item in self.goal_point:
                    # 删除上一个标点相关项目
                    self.canvas.delete(item)
                self.goal_point.clear()

    def goal_point_callback(self, msg: PoseStamped):
        if self.goal_point:
            for item in self.goal_point:
                # 删除上一个标点相关项目
                self.canvas.delete(item)
            self.goal_point.clear()
        if not self.is_goal_point.get():
            return
        # 画当前点
        rmuc_x, rmuc_y = msg.pose.position.x + 4.971, msg.pose.position.y + 7.5
        x, y = from_RMUC(rmuc_x, rmuc_y)

        circle = self.canvas.create_oval(x - 9, y - 9, x + 9, y + 9, fill="orange", outline="orange")
        label = self.canvas.create_text(x + (80 if x < 80 else -85 if x > 1800 - 80 else 0),
                                        y + (18 if y < 966 / 2 else -35), text=f"{rmuc_x:.3f}, {rmuc_y:.3f}",
                                        font=("Ubuntu Mono", 16), anchor='n')
        self.goal_point.extend([circle, label])

    # def autoaim_callback(self, autoaim_data: autoaim_to_decision):
    #     if self.autoaim_target:
    #         for item in self.autoaim_target:
    #             self.canvas.delete(item)
    #         self.autoaim_target.clear()
    #     if not self.is_autoaim.get():
    #         return
    #     for (idx, (r, angle)) in enumerate(zip(autoaim_data.polar_r_vector, autoaim_data.polar_angle_vector)):
    #         if r == .0 and angle == .0:
    #             continue
    #         rmuc_x, rmuc_y = (self.rmuc_cur_x + r * math.cos(math.radians(angle) + math.radians(self.cur_yaw)),
    #                           self.rmuc_cur_y + r * math.sin(math.radians(angle) + math.radians(self.cur_yaw)))
    #         x, y = from_RMUC(rmuc_x, rmuc_y)
    #         circle = self.canvas.create_oval(x - 5, y - 5, x + 5, y + 5, fill="red", outline="red")
    #         label = self.canvas.create_text(x + (80 if x < 80 else -85 if x > 1800 - 80 else 0),
    #                                         y + (18 if y < 966 / 2 else -35), text=f"{idx}\n{rmuc_x:.2f}, {rmuc_y:.2f}",
    #                                         font=("Ubuntu Mono", 14), anchor='n')
    #         self.autoaim_target.extend([circle, label])

    def autoaim_target_point_feedback_callback(self, points: Float64MultiArray):
        if self.autoaim_target:
            for item in self.autoaim_target:
                self.canvas.delete(item)
            self.autoaim_target.clear()
        if not self.is_autoaim.get():
            return
        autoaim_target_point = [points.data[i:i + 2] for i in range(0, 14, 2)]
        for (idx, (rmuc_x, rmuc_y)) in enumerate(autoaim_target_point):
            if rmuc_x == .0 and rmuc_y == .0:
                continue
            x, y = from_RMUC(rmuc_x, rmuc_y)
            hp = 0
            if idx < 4:
                hp = self.enemy_hp[idx]
            elif idx == 4:
                hp = self.enemy_outpost_hp
            elif idx == 5:
                hp = self.enemy_base_hp
            elif idx == 6:
                hp = self.enemy_hp[5]
            circle = self.canvas.create_oval(x - 5, y - 5, x + 5, y + 5, fill="red", outline="red")
            label = self.canvas.create_text(x + (80 if x < 80 else -85 if x > 1800 - 80 else 0),
                                            y + (18 if y < 966 / 2 else -35), text=f"{idx}  hp:{hp}\n{rmuc_x:.2f}, {rmuc_y:.2f}",
                                            font=("Ubuntu Mono", 14), anchor='n')
            self.autoaim_target.extend([circle, label])

    def autoaim_disappear_point_feedback_callback(self, points: Float64MultiArray):
        if self.autoaim_disappear:
            for item in self.autoaim_disappear:
                self.canvas.delete(item)
            self.autoaim_disappear.clear()
        if not self.is_autoaim.get():
            return
        autoaim_disappear_point = [points.data[i:i + 2] for i in range(0, 14, 2)]
        for (idx, (rmuc_x, rmuc_y)) in enumerate(autoaim_disappear_point):
            if rmuc_x == .0 and rmuc_y == .0:
                continue
            x, y = from_RMUC(rmuc_x, rmuc_y)
            hp = 0
            if idx < 4:
                hp = self.enemy_hp[idx]
            elif idx == 4:
                hp = self.enemy_outpost_hp
            elif idx == 5:
                hp = self.enemy_base_hp
            elif idx == 6:
                hp = self.enemy_hp[5]
            circle = self.canvas.create_oval(x - 5, y - 5, x + 5, y + 5, fill="grey", outline="grey")
            label = self.canvas.create_text(x + (80 if x < 80 else -85 if x > 1800 - 80 else 0),
                                            y + (18 if y < 966 / 2 else -35), text=f"{idx}  hp:{hp}\n{rmuc_x:.2f}, {rmuc_y:.2f}",
                                            font=("Ubuntu Mono", 14), anchor='n')
            self.autoaim_disappear.extend([circle, label])


if __name__ == "__main__":
    rclpy.init()

    node = rclpy.create_node("dummy_pursuit")
    pub = node.create_publisher(PointStamped, "pursuit_point", 10)

    root = tk.Tk()
    
    # 构造图片文件路径，优先查找RMUC.png，如果不存在则查找.pgm文件
    script_dir = os.path.dirname(os.path.abspath(__file__))
    possible_images = [
        os.path.join(script_dir, "RMUC.png"),
        os.path.join(script_dir, "QINGQING_PURSUIT.pgm"),
        os.path.join(script_dir, "..", "RMUC.png"),
        os.path.join(script_dir, "..", "QINGQING_PURSUIT.pgm")
    ]
    
    image_path = None
    for path in possible_images:
        if os.path.exists(path):
            image_path = path
            break
    
    if image_path is None:
        image_path = os.path.join(script_dir, "RMUC.png")  # 使用默认路径，即使文件不存在
    
    app = ImageAnnotator(root, image_path, node, pub)

    node.create_subscription(Odometry, "/state_estimation", app.refresh_cur_point, 1)
    node.create_subscription(send_data, "/serial_send_data", app.send_data_callback, 1)
    node.create_subscription(receive_data, "/serial_receive_data", app.receive_data_callback, 1)
    node.create_subscription(PoseStamped, "/move_base_simple/goal", app.goal_point_callback, 1)
    # node.create_subscription(autoaim_to_decision, "/autoaim_to_decision", app.autoaim_callback, 1)

    node.create_subscription(
        Float64MultiArray,
        "/autoaim_target_point",
        app.autoaim_target_point_feedback_callback,
        1,
    )
    node.create_subscription(
        Float64MultiArray,
        "/autoaim_disappear_point",
        app.autoaim_disappear_point_feedback_callback,
        1,
    )

    # 新建一个线程，用于ros订阅回调
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    root.mainloop()

    node.destroy_node()
    rclpy.shutdown()
