#!/usr/bin/env python3
"""task_cli: terminal tool that only SENDS commands on /task_cmd; every judgement is in task_executor (spec 6.5).

Interactive (focus this terminal):   ros2 run go2_task task_cli            ros2 run go2_task task_cli --load <name>
  c  collect: then publish /goal_pose goals (the web console's point tool does; each one = the next waypoint, checked at once)
  r  run once        l  run in a loop        u  undo last waypoint      x  clear all
  k  cancel the task (the dog stops; the REMOTE is the e-stop)
  s  save (asks for a name)                  o  load (asks for a name)      h  help      q  quit
One-shot:  ros2 run go2_task task_cli <command> [arg]     command: collect run_once run_loop undo clear cancel save load
/task_cmd is transient-local; each send waits until task_executor acknowledged it (up to 10 s), because a short-lived
publisher's command was lost before the reader side had matched (2026-10-25). The results come back as [task] events,
which are printed here.
"""
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.duration import Duration
from rclpy.signals import SignalHandlerOptions
from go2_msgs.msg import Event, TaskCmd

from go2_common import names
from go2_common.node import EVENT_QOS, TASK_CMD_QOS
from go2_common.events import format_event

COMMANDS = (TaskCmd.COLLECT, TaskCmd.RUN_ONCE, TaskCmd.RUN_LOOP, TaskCmd.UNDO, TaskCmd.CLEAR,
            TaskCmd.CANCEL, TaskCmd.SAVE, TaskCmd.LOAD)
KEYS = {'c': TaskCmd.COLLECT, 'r': TaskCmd.RUN_ONCE, 'l': TaskCmd.RUN_LOOP, 'u': TaskCmd.UNDO, 'x': TaskCmd.CLEAR,
        'k': TaskCmd.CANCEL, 's': TaskCmd.SAVE, 'o': TaskCmd.LOAD}


class Cli:
    def __init__(self):
        self.node = rclpy.create_node('task_cli')
        self.pub = self.node.create_publisher(TaskCmd, names.TASK_CMD, TASK_CMD_QOS)
        self.start_s = time.time()
        self.node.create_subscription(Event, names.event_topic('task'), self.on_event, EVENT_QOS)

    def on_event(self, m):
        if m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 < self.start_s - 1:
            return                                  # transient-local history from before this cli started
        print('  ' + format_event('task', m.name, {kv.key: kv.value for kv in m.fields}).replace('"', ''), flush=True)

    def send(self, command, arg=''):
        deadline = time.monotonic() + 5.0
        while self.pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        if self.pub.get_subscription_count() == 0:
            print('task_cli: no task_executor is listening on', names.TASK_CMD, '(is mode:=nav running?)', flush=True)
            return False
        msg = TaskCmd(command=command, arg=arg)
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.pub.publish(msg)
        acked = self.pub.wait_for_all_acked(Duration(seconds=10.0))
        print(f'task_cli: sent {command} {arg}'.rstrip() + ('' if acked else ' (NOT acknowledged within 10 s)'), flush=True)
        return acked


def read_line(prompt, settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    try:
        return input(prompt).strip()
    finally:
        tty.setcbreak(sys.stdin.fileno())


def interactive(cli, load_name):
    if not sys.stdin.isatty():
        print('task_cli: interactive mode needs a terminal; use the one-shot form instead', file=sys.stderr)
        return 2
    print(__doc__.split('One-shot')[0], flush=True)
    if load_name:
        cli.send(TaskCmd.LOAD, load_name)
    settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while rclpy.ok():
            rclpy.spin_once(cli.node, timeout_sec=0.05)
            if not select.select([sys.stdin], [], [], 0)[0]:
                continue
            k = sys.stdin.read(1)
            if k in ('q', '\x03'):
                break
            if k == 'h':
                print(__doc__.split('One-shot')[0], flush=True)
            elif k in KEYS:
                cmd = KEYS[k]
                arg = read_line('mission name: ', settings) if cmd in (TaskCmd.SAVE, TaskCmd.LOAD) else ''
                if cmd in (TaskCmd.SAVE, TaskCmd.LOAD) and not arg:
                    print('  (no name, nothing sent)')
                    continue
                cli.send(cmd, arg)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return 0


def main():
    args = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    load_name = ''
    if args[:1] == ['--load']:
        if len(args) < 2:
            print(__doc__)
            sys.exit(2)
        load_name, args = args[1], []
    if args and (args[0] not in COMMANDS or (args[0] in (TaskCmd.SAVE, TaskCmd.LOAD) and len(args) < 2)):
        print(__doc__)
        print('commands:', ' '.join(COMMANDS))
        sys.exit(2)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    cli = Cli()
    try:
        if args:
            ok = cli.send(args[0], args[1] if len(args) > 1 else '')
            end = time.monotonic() + 1.0                      # show the immediate result events
            while time.monotonic() < end:
                rclpy.spin_once(cli.node, timeout_sec=0.05)
            code = 0 if ok else 1
        else:
            code = interactive(cli, load_name)
    except KeyboardInterrupt:
        code = 0
    finally:
        cli.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
