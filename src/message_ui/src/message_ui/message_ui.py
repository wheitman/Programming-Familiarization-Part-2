#!/usr/bin/env python

# Copyright (c) 2011, Dorian Scholz, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import division
import os
from ament_index_python.packages import get_package_share_directory
import threading
import re

import rclpy
from mrsd_msgs.msg import ReplyMsg, SentMsg, ArithmeticReply
from mrsd_msgs.srv import Counter
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Slot
from python_qt_binding.QtGui import QKeySequence
from python_qt_binding.QtWidgets import QShortcut, QWidget
from rqt_gui_py.plugin import Plugin

OPERATOR_TYPES = ["ADD", "SUBTRACT", "MULTIPLY", "DIVIDE"]


class MessageUI(Plugin):

    def reply_msg_callback(self, msg_in):
        print(f"GOT REPLY: {msg_in.message}")
        self._widget.reply.setText(msg_in.message)

    def arithmetic_reply_msg_callback(self, msg_in):
        display_text = (
            "Operation Type: "
            + OPERATOR_TYPES[msg_in.oper_type]
            + "\n"
            + "Answer: "
            + str(msg_in.answer)
            + "\n"
            + "Time Received: "
            + str(msg_in.time_received)
            + "\n"
            + "Time Answered: "
            + str(msg_in.time_answered)
            + "\n"
            + "Process Time: "
            + str(msg_in.process_time)
        )
        self._widget.reply.setText(display_text)

    def message_count_display(self, counter_val):
        display_text = ""
        if self.counter_req_id == 0:
            display_text = display_text + "Total messages:"
        elif self.counter_req_id == 1:
            display_text = display_text + "Total replied messages:"
        elif self.counter_req_id == 2:
            display_text = display_text + "Total sent messages:"
        elif self.counter_req_id == 3:
            display_text = display_text + "Time since last replied message:"
        elif self.counter_req_id == 4:
            display_text = display_text + "Time since last sent message:"
        print(display_text)
        self._widget.counter_reply.setText(display_text + str(counter_val.reply))

    def __init__(self, context):
        super(MessageUI, self).__init__(context)
        self.setObjectName("MessageGUI")

        # Since we're in a Plugin class, we must create a node manually
        self.node = rclpy.create_node("message_ui_node")
        self.message_pub = self.node.create_publisher(SentMsg, "sent_msg", 10)
        self.node.create_subscription(
            ReplyMsg, "reply_msg", self.reply_msg_callback, 10
        )
        self.node.create_subscription(
            ArithmeticReply, "arithmetic_reply", self.arithmetic_reply_msg_callback, 10
        )

        self.counter_client = self.node.create_client(Counter, "message_counter")

        self.msg_to_send = SentMsg()
        self.counter_req_id = -1

        self._widget = QWidget()
        ui_file = os.path.join(
            get_package_share_directory("message_ui"), "resource", "MessageUI.ui"
        )
        loadUi(ui_file, self._widget)
        self._widget.setObjectName("MessageGUIui")
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (" (%d)" % context.serial_number())
            )
        context.add_widget(self._widget)

        self._widget.message_to_send.textChanged.connect(self._on_msg_to_send_changed)
        self._widget.counter_val_to_get.textChanged.connect(
            self._on_counter_val_to_get_changed
        )

        self._widget.send_message.pressed.connect(self._on_send_message_pressed)
        self._widget.send_request.pressed.connect(self._on_send_request_pressed)
        self.spin_node()

    def spin_node(self):
        """We have to spin the node manually, since the Plugin does not do this for us."""
        threading.Timer(0.1, self.spin_node).start()
        rclpy.spin_once(self.node, timeout_sec=0.05)

    def _on_msg_to_send_changed(self, msg):
        msg = str(msg)
        self.msg_to_send.message = msg

    def _on_send_message_pressed(self):
        self.msg_to_send.header.stamp = self.node.get_clock().now().to_msg()
        self.message_pub.publish(self.msg_to_send)

    def _on_counter_val_to_get_changed(self, msg):
        input_str = re.sub("[^0-9]", "", str(msg))
        self._widget.counter_val_to_get.setText(input_str)
        print(input_str)
        print(type(self._widget.counter_val_to_get))
        try:
            self.counter_req_id = int(input_str)
        except ValueError:
            self._widget.counter_val_to_get.clear()
            print(f'String input "{input_str}" is not an integer')

    def _on_send_request_pressed(self):
        try:
            request = Counter.Request()
            request.req_id = self.counter_req_id
            response = self.counter_client.call(request)
            self.message_count_display(response).reply
            return response
        except Exception as e:
            print(f"Service call to get message counter failed: {e}")

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
