#!/usr/bin/env python3
#
# Node adapted from https://github.com/ros2/rosbag2/pull/729#issuecomment-819747652
#
# Provides the built-in functionality of the rosbag package with the extension to
# repeat play next until a message on a selected topic is received

import rclpy
import time
import threading
from rclpy.node import Node
from ros2topic.api import get_msg_class
from getkey import getkey, keys
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from rosbag2_interfaces.srv import (
    Pause,
    Resume,
    TogglePaused,
    IsPaused,
    PlayNext,
    SetRate,
    GetRate
)


class RosbagPlayControllerNode(Node):
    def __init__(self):
        super().__init__('playcontrol')
        self.pause_client = self.create_client(Pause, '/rosbag2_player/pause')
        self.resume_client = self.create_client(Resume, '/rosbag2_player/resume')
        self.toggle_client = self.create_client(TogglePaused, '/rosbag2_player/toggle_paused')
        self.is_paused_client = self.create_client(IsPaused, '/rosbag2_player/is_paused')
        self.play_next_client = self.create_client(PlayNext, '/rosbag2_player/play_next')
        self.get_rate_client = self.create_client(GetRate, '/rosbag2_player/get_rate')
        self.set_rate_client = self.create_client(SetRate, '/rosbag2_player/set_rate')
        self.timer = self.create_timer(1, self.create_subscriptions)

        self.declare_parameter('topics', [''])
        self.topics = self.get_parameter('topics').value
        self.unsubscribed_topics = self.topics

        self.message_received = False
        self.last_received_topic = ''

        self.print_usage()

        self.get_logger().info("Waiting until all topics are subscribed...")

    def create_subscriptions(self):
        # cancel timer to use it as a one-shot one
        self.timer.cancel()

        subscribed_topics = []

        for topic in self.unsubscribed_topics:
            if len(topic) > 0:
                message_type = get_msg_class(self, topic, include_hidden_topics=True)

                # rosbag is not running yet --> not topic info available
                if message_type is None:
                    continue

                qos_profile = QoSProfile(
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    depth=1
                )

                self.create_subscription(message_type, topic, lambda m: self.on_message_received(topic), qos_profile)
                subscribed_topics.append(topic)

                self.get_logger().info(f"Subscribed to topic \"{topic}\"")

        # remove all successfully subscribed topics
        for topic in subscribed_topics:
            self.unsubscribed_topics.remove(topic)

        # stop timer if all needed topics are subscribed
        if len(self.unsubscribed_topics) > 0:
            self.timer.reset()
        else:
            self.destroy_timer(self.timer)
            self.get_logger().info("Subscribed to all needed topics!")

    def on_message_received(self, topic):
        self.message_received = True
        self.last_received_topic = topic

    def print_usage(self):
        usage = "ROSBag Play Controller\n" \
                "Press SPACE for Toggle Pause/Resume\n" \
                "Press P for Pause\n" \
                "Press R for Resume\n" \
                "Press CURSOR_RIGHT for Play Next Message\n" \
                "Press N for Play Until Message Sent On Specified Topic(s)\n" \
                "Press CURSOR_UP for Increase Rate by 0.1\n" \
                "Press CURSOR_DOWN for Decrease Rate by 0.1\n" \
                "Press Q to shut the node down"

        self.get_logger().info(usage)

    def print_state(self, future):
        res = future.result()
        if res is not None:
            self.get_logger().info(f'IsPaused: {res.paused}', once=True)
        else:
            self.get_logger().error(f'Exception while calling service: {future.exception()}')

    def trigger_result_cb(self, name, additional_text = None, recursive_play_next=False):
        def impl(future):
            res = future.result()
            if res is not None:
                if recursive_play_next and not self.message_received:
                    req = PlayNext.Request()
                    future = self.play_next_client.call_async(req)
                    future.add_done_callback(self.trigger_result_cb('PlayNextTopic', recursive_play_next=True))
                    return

                self.print_callback_info(name,  additional_text)
            else:
                self.get_logger().error(f'Exception for service: {future.exception()}')

        return impl

    def print_callback_info(self, callback_name, additional_text):
        if callback_name == 'Pause':
            self.get_logger().info('Paused playback.')
        elif callback_name == 'Resume':
            self.get_logger().info('Resumed playback.')
        elif callback_name == 'Toggle':
            self.get_logger().info('Toggled playback state.')
        elif callback_name == 'PlayNext':
            self.get_logger().info('Played the next message.')
        elif callback_name == 'PlayNextTopic':
            self.get_logger().info(f'Played until the next message for topic \"{self.last_received_topic}\"')
        elif callback_name == 'UpdateRate':
            self.get_logger().info(f'Set replay rate to {additional_text}')
        else:
            self.get_logger().info(f'Received response for callback \"{callback_name}\"')

    def state_timer(self):
        req = IsPaused.Request()
        future = self.is_paused_client.call_async(req)
        future.add_done_callback(self.print_state)

    def pause(self):
        req = Pause.Request()
        future = self.pause_client.call_async(req)
        future.add_done_callback(self.trigger_result_cb('Pause'))

    def resume(self):
        req = Resume.Request()
        future = self.resume_client.call_async(req)
        future.add_done_callback(self.trigger_result_cb('Resume'))

    def toggle_paused(self):
        req = TogglePaused.Request()
        future = self.toggle_client.call_async(req)
        future.add_done_callback(self.trigger_result_cb('Toggle'))

    def play_next(self):
        req = PlayNext.Request()
        future = self.play_next_client.call_async(req)
        future.add_done_callback(self.trigger_result_cb('PlayNext'))

    def play_next_topic(self):
        self.message_received = False
        self.message_received = None
        req = PlayNext.Request()
        future = self.play_next_client.call_async(req)
        future.add_done_callback(self.trigger_result_cb('PlayNextTopic', recursive_play_next=True))

    def increase_rate(self):
        req = GetRate.Request()
        future = self.get_rate_client.call_async(req)
        future.add_done_callback(lambda f: self.rate_received_cb(f, rate_change=0.1))

    def decrease_rate(self):
        req = GetRate.Request()
        future = self.get_rate_client.call_async(req)
        future.add_done_callback(lambda f: self.rate_received_cb(f, rate_change=-0.1))

    def rate_received_cb(self, future, rate_change):
        cur_rate = future.result().rate
        new_rate = max(0.1, cur_rate + rate_change)

        req = SetRate.Request()
        req.rate = new_rate
        future = self.set_rate_client.call_async(req)
        future.add_done_callback(self.trigger_result_cb('UpdateRate', f'{new_rate:.1f}'))

def spin_fn(node):
    node.get_logger().info('Starting spin thread')
    rclpy.spin(node)
    node.get_logger().info('Spin thread done')


def main():
    rclpy.init()
    node = RosbagPlayControllerNode()
    spin_thread = threading.Thread(target=spin_fn, args=[node])
    spin_thread.start()

    try:
        while True:
            c = getkey()

            if c == keys.SPACE:
                node.toggle_paused()
            elif c == keys.P:
                node.pause()
            elif c == keys.R:
                node.resume()
            elif c == keys.RIGHT:
                node.play_next()
            elif c == keys.UP:
                node.increase_rate()
            elif c == keys.DOWN:
                node.decrease_rate()
            elif c == keys.N:
                node.play_next_topic()
            elif c == keys.ESCAPE or c == keys.Q:
                break
            else:
                print(f'Ignored: {c}')
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()
