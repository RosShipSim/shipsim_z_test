#! /usr/bin/python3
# -*- coding: utf-8 -*-

import sys
import numpy as np

import rclpy
from rclpy.node import Node

from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtWidgets import QApplication, QDialog, QWidget

from shipsim_msgs_module.msg import MMGControl
from shipsim_msgs_module.msg import PositionSensor

from shipsim_z_test.ZTestMMG import Ui_ZTestMMG


class ZTestNode(Node):
    """ControllerNode."""

    def __init__(self):
        """init."""
        super().__init__("ztest", namespace="ship1")
        self.declare_parameter("publish_address1", "/ship1/z_test_control")

        publish_address1 = (
            self.get_parameter("publish_address1").get_parameter_value().string_value
        )
        self.publisher1 = self.create_publisher(MMGControl, publish_address1, 1)


class ControllerNodeWorker(QThread):
    """ControllerNodeWorker."""

    signal = pyqtSignal()
    pose_now = PositionSensor()
    cri_angle1 = 0
    cri_angle2 = 0

    def __init__(self):
        """init."""
        super(ControllerNodeWorker, self).__init__()
        self.control_msg = MMGControl()
        self.rate = 1.0

    def set_control_msg(self, msg):
        """set_control_msg."""
        self.control_msg = msg

    ### ポジションセンサからデータ受け取りと更新
    def listener_callback(self, msg):
        """listener_callback."""
        #self.node.get_logger().info(
        #    'z test Node heard: "%s","%s","%s"  '
        #    % (msg.x, msg.y, msg.psi)
        #)
        self.pose_now.psi = msg.psi * 180 / np.pi
        print(self.pose_now.psi)

    def run(self):
        """run."""
        rclpy.init()
        self.node = ZTestNode()
        self.node.create_rate(self.rate)
        
        i = 0

        while rclpy.ok():
            rclpy.spin_once(self.node)
            
            ### ポジションセンサからデータ受け取り
            self.node.subscription = self.node.create_subscription(
                PositionSensor, "/ship1/PositionSensor", self.listener_callback, 1
            )

            ### 基準角設定
            if i == 0:
                self.cri_angle1 = self.pose_now.psi + self.control_msg.rudder_angle_degree
                self.cri_angle2 = self.pose_now.psi - self.control_msg.rudder_angle_degree
                print(self.cri_angle1)
                print(self.cri_angle2)
                i = 1
            
            ### 船首角の判別
            # 最初
            if self.pose_now.psi < self.cri_angle1 and i == 1:
                self.control_msg.rudder_angle_degree = self.control_msg.rudder_angle_degree
            # 転舵1
            if self.pose_now.psi >= self.cri_angle1 and i == 1:
                self.control_msg.rudder_angle_degree = -(self.control_msg.rudder_angle_degree)
                i = 2
            # 転舵2
            if self.pose_now.psi <= self.cri_angle2 and i ==2:
                self.control_msg.rudder_angle_degree = -(self.control_msg.rudder_angle_degree)
                i = 3

            print(i)

            ### 舵角の送信
            self.node.publisher1.publish(self.control_msg)
            self.node.get_logger().info(
                'Publishing: "%s", "%s"'
                % (self.control_msg.n_p, self.control_msg.rudder_angle_degree)
            )
        else:
            rclpy.shutdown()
            # self.control_msg = MMGControl()


class ControllerUi(QDialog): ###コントローラーからの入力を受け取る
    """ControllerUI."""

    worker_thread = ControllerNodeWorker()

    def __init__(self, parent=None):
        """init."""
        super(ControllerUi, self).__init__(parent)
        centralWidget = QWidget(self)
        self.ui = Ui_ZTestMMG()
        self.ui.setupUi(self)
        self.sampling_freq = 1.0

    def clicked_start(self):
        """clicked start button"""
        control_msg = self.worker_thread.control_msg
        self.worker_thread.rate = 1.0
        control_msg.n_p = float(self.ui.ZpropellerSlider.value())
        self.ui.ZpropellerSlider.setEnabled(False)
        self.ui.ZpropellerBox.setEnabled(False)
        control_msg.rudder_angle_degree = float(self.ui.ZrudderBox.value())
        self.ui.ZrudderBox.setEnabled(False)
        self.worker_thread.start()

    def change_shaft_revolution(self):
        """change shaft revolution slider"""
        n_p = self.ui.ZpropellerSlider.value()
        control_msg = self.worker_thread.control_msg
        control_msg.n_p = float(n_p)

    def change_rudder_angle(self):
        """change rudder angle dial"""
        rudder_angle_degree = self.ui.ZrudderBox.value()
        control_msg = self.worker_thread.control_msg
        control_msg.rudder_angle_degree = float(rudder_angle_degree)
        

def main(args=None):
    """Run main."""
    app = QApplication(sys.argv)
    window = ControllerUi()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
