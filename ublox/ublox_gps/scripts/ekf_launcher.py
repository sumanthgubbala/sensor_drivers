#!/usr/bin/env python3
import rospy
import subprocess
import shlex
import os
import signal
import time
import sys
from distutils.spawn import find_executable

def safe_terminate_process(proc, timeout=3.0):
    if proc is None:
        return
    if proc.poll() is not None:
        return
    try:
        proc.send_signal(signal.SIGINT)
    except Exception:
        pass
    wait_until = time.time() + timeout
    while time.time() < wait_until:
        if proc.poll() is not None:
            return
        time.sleep(0.1)
    try:
        proc.terminate()
    except Exception:
        pass
    wait_until = time.time() + timeout
    while time.time() < wait_until:
        if proc.poll() is not None:
            return
        time.sleep(0.1)
    try:
        proc.kill()
    except Exception:
        pass

def launch_roslaunch(pkg, launchfile, extra_args="", use_xterm=False, xterm_title="ekf.launch"):
    roslaunch_exec = find_executable('roslaunch')
    if roslaunch_exec is None:
        raise RuntimeError("roslaunch not found")
    base_cmd = f"roslaunch {pkg} {launchfile} {extra_args}".strip()
    if use_xterm:
        xterm_exec = find_executable('xterm')
        if xterm_exec:
            cmd = f"{xterm_exec} -T {shlex.quote(xterm_title)} -hold -e {shlex.quote(base_cmd)}"
            proc = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)
            return proc
    args = shlex.split(base_cmd)
    proc = subprocess.Popen(args)
    return proc

class EkfSupervisor:
    def __init__(self):
        self.startup_wait_s = float(rospy.get_param('/ekf_launcher/startup_wait_s', 10.0))
        self.check_interval = float(rospy.get_param('/ekf_launcher/check_interval_s', 0.5))
        self.failure_debounce_s = float(rospy.get_param('/ekf_launcher/failure_debounce_s', 1.0))
        self.use_xterm = bool(rospy.get_param('/ekf_launcher/use_xterm', False))
        self.xterm_title = rospy.get_param('/ekf_launcher/xterm_title', 'ekf.launch')
        self.roslaunch_pkg = rospy.get_param('/ekf_launcher/roslaunch_pkg', 'ublox_gps')
        self.roslaunch_file = rospy.get_param('/ekf_launcher/roslaunch_file', 'ekf.launch')
        self.roslaunch_args = rospy.get_param('/ekf_launcher/roslaunch_args', '')
        self.proc = None
        self._shutting_down = False
        rospy.on_shutdown(self.shutdown)

    def start_launch(self):
        try:
            self.proc = launch_roslaunch(self.roslaunch_pkg, self.roslaunch_file,
                                         extra_args=self.roslaunch_args,
                                         use_xterm=self.use_xterm,
                                         xterm_title=self.xterm_title)
            time.sleep(0.25)
        except Exception as e:
            rospy.logerr("Failed to start roslaunch: %s", e)
            self.proc = None

    def stop_launch(self):
        if self.proc is None:
            return
        safe_terminate_process(self.proc, timeout=2.0)
        try:
            if self.proc.stdout:
                self.proc.stdout.close()
            if self.proc.stderr:
                self.proc.stderr.close()
        except Exception:
            pass
        self.proc = None

    def is_proc_alive(self):
        return (self.proc is not None) and (self.proc.poll() is None)

    def wait_for_localization_valid(self, timeout_s):
        deadline = time.time() + timeout_s
        while time.time() < deadline and not rospy.is_shutdown():
            try:
                val = rospy.get_param('/localization_valid', False)
            except Exception:
                val = False
            if val:
                return True
            if not self.is_proc_alive():
                return False
            time.sleep(self.check_interval)
        return False

    def monitor_loop(self):
        while not rospy.is_shutdown() and not self._shutting_down:
            self.start_launch()
            if self.proc is None:
                time.sleep(2.0)
                continue
            ok = self.wait_for_localization_valid(self.startup_wait_s)
            if not ok:
                self.stop_launch()
                time.sleep(0.5)
                continue
            while not rospy.is_shutdown() and self.is_proc_alive() and not self._shutting_down:
                try:
                    val = rospy.get_param('/localization_valid', False)
                except Exception:
                    val = False
                if val:
                    time.sleep(self.check_interval)
                    continue
                deb_deadline = time.time() + self.failure_debounce_s
                still_bad = True
                while time.time() < deb_deadline and not rospy.is_shutdown() and self.is_proc_alive():
                    try:
                        val2 = rospy.get_param('/localization_valid', False)
                    except Exception:
                        val2 = False
                    if val2:
                        still_bad = False
                        break
                    time.sleep(self.check_interval)
                if still_bad:
                    self.stop_launch()
                    time.sleep(0.5)
                    break
            if not self.is_proc_alive():
                self.stop_launch()
                time.sleep(0.5)
                continue

    def shutdown(self):
        self._shutting_down = True
        try:
            self.stop_launch()
        except Exception:
            pass

def main():
    rospy.init_node('ekf_launch_supervisor', anonymous=False)
    sup = EkfSupervisor()
    try:
        sup.monitor_loop()
    except KeyboardInterrupt:
        pass
    except rospy.ROSInterruptException:
        pass
    finally:
        sup.shutdown()

if __name__ == '__main__':
    main()
