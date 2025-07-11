#!/usr/bin/env python3
"""
GBX Predict Standalone Application - 优化横向长图像显示
独立的图像显示和电机控制应用程序 - 优化版本
"""

import sys
import os
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from control_msgs.msg import MultiDOFCommand
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue
from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import Optional

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget,
    QLabel, QComboBox, QPushButton, QTabWidget, QGridLayout, QFrame,
    QGroupBox, QMessageBox
)
from PyQt5.QtCore import QTimer, pyqtSignal, QThread, QObject, Qt
from PyQt5.QtGui import QPixmap, QImage

class ImageSubscriber(QThread):
    """图像订阅线程"""
    image_received = pyqtSignal(np.ndarray)
    
    def __init__(self, node: Node, topic_name: str):
        super().__init__()
        self.node = node
        self.topic_name = topic_name
        self.bridge = CvBridge()
        self.subscription = None
        self.latest_image = None
        
    def run(self):
        # 创建订阅
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.subscription = self.node.create_subscription(
            Image,
            self.topic_name,
            self.image_callback,
            qos_profile
        )
        
    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received.emit(cv_image)
        except Exception as e:
            self.node.get_logger().error(f"图像转换失败: {e}")
    
    def stop(self):
        if self.subscription:
            self.node.destroy_subscription(self.subscription)
        self.quit()
        self.wait()

class GbxPredictApp(QMainWindow):
    """主应用程序窗口"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("GBX Predict 控制面板")
        self.setGeometry(100, 100, 1400, 900)  # 增加窗口宽度以适应长图像
        
        # 初始化ROS2
        rclpy.init()
        self.node = Node('gbx_predict_standalone_corrected')
        
        # 创建发布者
        self.motor_publisher = self.node.create_publisher(
            MultiDOFCommand, 
            '/velocity_controller/reference', 
            10
        )
        
        # 创建参数客户端
        self.param_client = self.node.create_client(
            SetParameters,
            '/stitching_node/set_parameters'
        )
        
        # 图像订阅器字典
        self.image_subscribers = {}
        self.image_labels = {}
        
        # 显示旋转标志
        self.rotate_display = False
        
        # 初始化UI
        self.init_ui()
        
        # ROS2 spinner线程
        self.ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.ros_thread.start()
        
        # 调试信息
        print("应用程序初始化完成")
        print(f"电机发布者已创建: {self.motor_publisher}")
        
    def init_ui(self):
        """初始化用户界面"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 创建标签页
        tab_widget = QTabWidget()
        central_widget.setLayout(QVBoxLayout())
        central_widget.layout().addWidget(tab_widget)
        
        # Deploy页面
        deploy_tab = self.create_deploy_tab()
        tab_widget.addTab(deploy_tab, "Deploy")
        
        # Debug页面
        debug_tab = self.create_debug_tab()
        tab_widget.addTab(debug_tab, "Debug")
        
    def create_deploy_tab(self):
        """创建Deploy页面 - 优化长图像显示"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # 图像显示区域
        image_group = QGroupBox("图像显示")
        image_layout = QVBoxLayout(image_group)
        
        # 话题选择
        topic_layout = QHBoxLayout()
        topic_layout.addWidget(QLabel("图像话题:"))
        self.deploy_topic_combo = QComboBox()
        self.deploy_topic_combo.addItems([
            "/continuous_stitched_image",
            "/stitched_image", 
            "/hk_camera/left_camera/image_raw",
            "/hk_camera/right_camera/image_raw"
        ])
        # 连接信号 - 类型注释确保正确识别
        self.deploy_topic_combo.currentTextChanged.connect(self.on_deploy_topic_changed)  # type: ignore
        topic_layout.addWidget(self.deploy_topic_combo)
        image_layout.addLayout(topic_layout)
        
        # 图像显示标签 - 专为横向长图像优化
        self.deploy_image_label = QLabel("等待图像...")
        self.deploy_image_label.setStyleSheet("""
            border: 2px solid #ddd; 
            min-height: 500px; 
            max-height: 500px;
            background-color: #f8f8f8;
            padding: 5px;
        """)
        self.deploy_image_label.setScaledContents(False)  # 禁用自动缩放
        image_layout.addWidget(self.deploy_image_label)
        
        layout.addWidget(image_group)
        
        # 电机控制区域
        motor_group = QGroupBox("电机控制")
        motor_layout = QHBoxLayout(motor_group)
        
        # 前进按钮
        self.forward_btn = QPushButton("前进 ▲")
        self.forward_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-size: 14px; font-weight: bold; }")
        self.forward_btn.clicked.connect(self.send_forward_command)  # type: ignore
        motor_layout.addWidget(self.forward_btn)
        
        # 停止按钮
        self.stop_btn = QPushButton("停止 ■")
        self.stop_btn.setStyleSheet("QPushButton { background-color: #f44336; color: white; font-size: 14px; font-weight: bold; }")
        self.stop_btn.clicked.connect(self.send_stop_command)  # type: ignore
        motor_layout.addWidget(self.stop_btn)
        
        # 后退按钮
        self.backward_btn = QPushButton("后退 ▼")
        self.backward_btn.setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-size: 14px; font-weight: bold; }")
        self.backward_btn.clicked.connect(self.send_backward_command)  # type: ignore
        motor_layout.addWidget(self.backward_btn)
        
        layout.addWidget(motor_group)
        
        # 拼接控制区域
        stitch_group = QGroupBox("拼接控制")
        stitch_layout = QVBoxLayout(stitch_group)
        
        # 显示方向控制
        direction_layout = QHBoxLayout()
        direction_layout.addWidget(QLabel("显示方向:"))
        
        self.normal_display_btn = QPushButton("正常显示")
        self.normal_display_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-size: 12px; }")
        self.normal_display_btn.clicked.connect(lambda: self.set_display_rotation(False))  # type: ignore
        direction_layout.addWidget(self.normal_display_btn)
        
        self.rotate_display_btn = QPushButton("旋转90°显示")
        self.rotate_display_btn.setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-size: 12px; }")
        self.rotate_display_btn.clicked.connect(lambda: self.set_display_rotation(True))  # type: ignore
        direction_layout.addWidget(self.rotate_display_btn)
        
        stitch_layout.addLayout(direction_layout)
        
        # 重置按钮
        reset_layout = QHBoxLayout()
        self.reset_btn = QPushButton("重置拼接")
        self.reset_btn.setStyleSheet("QPushButton { background-color: #FF9800; color: white; font-size: 14px; font-weight: bold; }")
        self.reset_btn.clicked.connect(self.reset_stitching)  # type: ignore
        reset_layout.addWidget(self.reset_btn)
        
        stitch_layout.addLayout(reset_layout)
        
        layout.addWidget(stitch_group)
        
        # 启动默认图像订阅
        self.on_deploy_topic_changed(self.deploy_topic_combo.currentText())
        
        return widget
        
    def create_debug_tab(self):
        """创建Debug页面"""
        widget = QWidget()
        layout = QGridLayout(widget)
        
        # 创建4个监控窗口
        topics = [
            "/hk_camera/left_camera/image_raw",
            "/hk_camera/right_camera/image_raw", 
            "/debug_image",
            "/stitched_image"
        ]
        
        for i, topic in enumerate(topics):
            row = i // 2
            col = i % 2
            
            # 创建监控组
            group = QGroupBox(f"监控窗口 {i+1}")
            group_layout = QVBoxLayout(group)
            
            # 话题选择
            topic_layout = QHBoxLayout()
            topic_layout.addWidget(QLabel("话题:"))
            combo = QComboBox()
            combo.addItems([
                "/debug_image",
                "/stitched_image",
                "/hk_camera/left_camera/image_raw", 
                "/hk_camera/right_camera/image_raw"
            ])
            combo.setCurrentText(topic)
            # 使用 lambda 来传递参数，并添加类型注释
            combo.currentTextChanged.connect(lambda text, idx=i: self.on_debug_topic_changed(idx, text))  # type: ignore
            topic_layout.addWidget(combo)
            group_layout.addLayout(topic_layout)
            
            # 图像显示
            image_label = QLabel("等待图像...")
            image_label.setStyleSheet("border: 1px solid gray; min-height: 200px;")
            image_label.setScaledContents(True)
            group_layout.addWidget(image_label)
            
            self.image_labels[f"debug_{i}"] = image_label
            
            layout.addWidget(group, row, col)
            
            # 启动订阅
            self.on_debug_topic_changed(i, topic)
        
        return widget
    
    def send_forward_command(self):
        """发送前进命令 - 使用正确的消息格式"""
        print("前进按钮被点击!")
        # 根据alias: photo_station_v2_move='ros2 topic pub -r 10 /velocity_controller/reference control_msgs/msg/MultiDOFCommand "{dof_names: ['joint1', 'joint2'], values: [-0.5, 0.5], values_dot: []}"'
        self.send_motor_command_correct(['joint1', 'joint2'], [-0.5, 0.5], [])
    
    def send_stop_command(self):
        """发送停止命令 - 使用正确的消息格式"""
        print("停止按钮被点击!")
        # 根据alias: photo_station_v2_stop='ros2 topic pub -r 10 /velocity_controller/reference control_msgs/msg/MultiDOFCommand "{dof_names: ['joint1', 'joint2'], values: [0.0, 0.0], values_dot: []}"'
        self.send_motor_command_correct(['joint1', 'joint2'], [0.0, 0.0], [])
    
    def send_backward_command(self):
        """发送后退命令 - 使用正确的消息格式"""
        print("后退按钮被点击!")
        # 根据alias: photo_station_v2_back='ros2 topic pub -r 10 /velocity_controller/reference control_msgs/msg/MultiDOFCommand "{dof_names: ['joint1', 'joint2'], values: [2.0, -2.0], values_dot: []}"'
        self.send_motor_command_correct(['joint1', 'joint2'], [2.0, -2.0], [])
    
    def on_deploy_topic_changed(self, topic_name):
        """Deploy页面话题变更"""
        print(f"Deploy话题变更为: {topic_name}")
        self.start_image_subscription("deploy", topic_name, self.deploy_image_label)
    
    def on_debug_topic_changed(self, window_idx, topic_name):
        """Debug页面话题变更"""
        print(f"Debug窗口{window_idx}话题变更为: {topic_name}")
        key = f"debug_{window_idx}"
        if key in self.image_labels:
            self.start_image_subscription(key, topic_name, self.image_labels[key])
    
    def start_image_subscription(self, key, topic_name, label):
        """启动图像订阅"""
        # 停止旧的订阅
        if key in self.image_subscribers:
            self.image_subscribers[key].stop()
            del self.image_subscribers[key]
        
        # 创建新的订阅
        subscriber = ImageSubscriber(self.node, topic_name)
        subscriber.image_received.connect(lambda img: self.update_image_display(label, img, key))
        subscriber.start()
        self.image_subscribers[key] = subscriber
        
        label.setText(f"订阅: {topic_name}")
        print(f"开始订阅话题: {topic_name}")
    
    def update_image_display(self, label, cv_image, key=""):
        """更新图像显示 - 根据不同类型优化显示，支持旋转"""
        try:
            # 获取标签大小
            label_width = label.width() if label.width() > 0 else 800
            label_height = label.height() if label.height() > 0 else 300
            
            # 获取图像尺寸
            img_height, img_width = cv_image.shape[:2]
            
            # 对于Deploy页面，如果启用了旋转显示，先旋转图像
            if key == "deploy" and self.rotate_display:
                # 逆时针旋转90度（将纵向长图变成横向显示）
                cv_image = cv2.rotate(cv_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
                img_height, img_width = cv_image.shape[:2]
            
            # 对于Deploy页面的连续拼接图像，使用特殊处理
            if key == "deploy":
                # 固定高度为标签高度的80%，宽度按比例缩放
                target_height = int(label_height * 0.9)
                scale = target_height / img_height
                target_width = int(img_width * scale)
                
                # 如果缩放后的宽度超过标签宽度，则限制宽度
                if target_width > label_width:
                    scale = label_width / img_width
                    target_width = int(img_width * scale)
                    target_height = int(img_height * scale)
                
                resized_image = cv2.resize(cv_image, (target_width, target_height))
            else:
                # Debug页面使用原来的逻辑
                scale_w = label_width / img_width
                scale_h = label_height / img_height
                scale = min(scale_w, scale_h)
                
                new_width = int(img_width * scale)
                new_height = int(img_height * scale)
                resized_image = cv2.resize(cv_image, (new_width, new_height))
            
            # 转换为Qt格式
            height, width, channel = resized_image.shape
            bytes_per_line = 3 * width
            q_image = QImage(resized_image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            pixmap = QPixmap.fromImage(q_image)
            
            # 设置pixmap
            label.setPixmap(pixmap)
            
        except Exception as e:
            self.node.get_logger().error(f"图像显示更新失败: {e}")
    
    def send_motor_command_correct(self, dof_names, values, values_dot):
        """发送正确格式的电机控制命令"""
        try:
            cmd = MultiDOFCommand()
            cmd.dof_names = dof_names
            cmd.values = values
            cmd.values_dot = values_dot
            
            self.motor_publisher.publish(cmd)
            print(f"发送电机命令成功:")
            print(f"  dof_names: {dof_names}")
            print(f"  values: {values}")
            print(f"  values_dot: {values_dot}")
            self.node.get_logger().info(f"发送电机命令: dof_names={dof_names}, values={values}, values_dot={values_dot}")
                
        except Exception as e:
            error_msg = f"电机控制失败: {e}"
            print(error_msg)
            self.node.get_logger().error(error_msg)
            QMessageBox.warning(self, "错误", error_msg)
    
    def set_display_rotation(self, rotate):
        """设置显示旋转"""
        self.rotate_display = rotate
        if rotate:
            print("设置为旋转90°显示")
            QMessageBox.information(self, "显示设置", "已设置为旋转90°显示\n纵向长图将横向显示")
        else:
            print("设置为正常显示")
            QMessageBox.information(self, "显示设置", "已设置为正常显示")
    
    def reset_stitching(self):
        """重置拼接 - 使用参数设置方式"""
        try:
            print("重置拼接按钮被点击!")
            
            # 尝试通过参数设置来重置拼接
            if not self.param_client.wait_for_service(timeout_sec=2.0):
                print("警告: 拼接服务不可用，尝试使用ros2 param命令")
                # 使用系统命令作为备选方案
                import subprocess
                try:
                    result = subprocess.run([
                        'ros2', 'param', 'set', '/stitching_node', 'reset_now', 'true'
                    ], capture_output=True, text=True, timeout=5)
                    
                    if result.returncode == 0:
                        QMessageBox.information(self, "成功", "拼接重置成功")
                        print("通过命令行重置拼接成功")
                    else:
                        QMessageBox.warning(self, "错误", f"重置失败: {result.stderr}")
                        print(f"命令行重置失败: {result.stderr}")
                except subprocess.TimeoutExpired:
                    QMessageBox.warning(self, "错误", "重置命令超时")
                    print("重置命令超时")
                except Exception as cmd_e:
                    QMessageBox.warning(self, "错误", f"命令执行失败: {cmd_e}")
                    print(f"命令执行失败: {cmd_e}")
                return
            
            request = SetParameters.Request()
            param = Parameter()
            param.name = "reset_now"  # 使用正确的参数名
            param.value = ParameterValue(type=1, bool_value=True)  # PARAMETER_BOOL = 1
            request.parameters = [param]
            
            future = self.param_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            
            if future.result():
                QMessageBox.information(self, "成功", "拼接重置成功")
                self.node.get_logger().info("拼接重置成功")
                print("拼接重置成功")
            else:
                QMessageBox.warning(self, "错误", "拼接重置失败")
                print("拼接重置失败")
        except Exception as e:
            error_msg = f"拼接重置失败: {e}"
            print(error_msg)
            self.node.get_logger().error(error_msg)
            QMessageBox.warning(self, "错误", error_msg)
    
    def spin_ros(self):
        """ROS2 spinner线程"""
        try:
            rclpy.spin(self.node)
        except Exception as e:
            print(f"ROS spinner错误: {e}")
    
    def closeEvent(self, event):
        """窗口关闭事件"""
        try:
            print("应用程序正在关闭...")
            # 停止所有图像订阅
            for subscriber in self.image_subscribers.values():
                subscriber.stop()
            
            # 关闭ROS2
            if self.node:
                self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"关闭时错误: {e}")
        
        event.accept()

def main():
    """主函数"""
    app = QApplication(sys.argv)
    
    # 创建并显示主窗口
    window = GbxPredictApp()
    window.show()
    
    # 运行应用
    try:
        print("应用程序启动...")
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        print("应用被用户中断")
    except Exception as e:
        print(f"应用错误: {e}")

if __name__ == "__main__":
    main() 