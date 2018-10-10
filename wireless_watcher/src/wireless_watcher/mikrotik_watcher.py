# Software License Agreement (BSD License)
#
#  Copyright (c) 2018, Locus Robotics
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  # Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  # Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#  # Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

import rospy
from std_msgs.msg import Bool
import subprocess
from wireless_watcher.mikrotik_api import RouterOSApi
from wireless_msgs.msg import Connection


def stripRes(results):
    return [res[1] for res in results if res[0] == "!re"]


def solveBitrate(hr):
    return sum([float(res.split("Mb")[0]) for res in [hr["=tx-rate"], hr["=rx-rate"]]])


class MikrotikWatcher(RouterOSApi):
    def __init__(self, hostname, port, user, psk, chip, default_tx_power=27):
        super(MikrotikWatcher, self).__init__(hostname, port)
        self._connected_publisher = rospy.Publisher("connected", Bool, queue_size=1)
        if not self.login(user, psk, verbose=False):
            raise IOError("Failed login to device {u}@{h}:{p} with psk: {k}".format(i=user, h=hostname, p=port, s=psk))
        self._connected_publisher.publish(True)
        self._connection_publisher = rospy.Publisher("connection", Connection, queue_size=1)
        self._chip = chip
        self._firmware = self.getFirmwareVersion()
        self._default_tx_power = default_tx_power
        self._kernel = subprocess.check_output(["uname", "-r"], stderr=subprocess.STDOUT).strip("\n")

    def talkResults(self, command_list, verbose=False):
        return [res[1] for res in self.talk(command_list, verbose=verbose) if res[0] == "!re"]

    def silentGet(self, command_list):
        return self.talkResults(command_list, verbose=False)

    def getAllInterfaces(self):
        return self.silentGet(["/interface/getall"])

    def getWirelessInterfaces(self):
        return self.silentGet(["/interface/print", "?type=wlan"])

    def getEthernetInterfaces(self):
        return self.silentGet(["/interface/print", "?type=ether"])

    def monitorWirelessInterface(self, interface="wlan1"):
        return self.silentGet(["/interface/wireless/monitor", "=numbers={}".format(interface), "=once="])

    def getFirmwareVersion(self):
        res = self.silentGet(["/system/package/getall"])
        for data in res:
            version = data.get("=version", None)
            if version is not None:
                break
        if version is None:
            version = 'unknown'
            rospy.logwarn("No firmware version could be identified, setting to '{}'".format(version))
        return version

    def publishNetworkHealth(self, interface, frame="mikrotik"):
        health_results = self.monitorWirelessInterface(interface)
        if len(health_results) != 1:
            raise ValueError("Health results not expected length")
        health_result = health_results[0]
        health_report = Connection()
        health_report.header.stamp = rospy.get_rostime()
        health_report.header.frame_id = frame
        health_report.wifi_chip = self._chip
        health_report.firmware_version = self._firmware
        health_report.kernel_version = self._kernel
        health_report.bssid = health_result["=bssid"]
        health_report.essid = health_result["=ssid"]
        health_report.signal_level = int(health_result["=signal-strength"])
        health_report.noise_level = int(health_result["=noise-floor"])
        link_quality = float(health_result["=tx-ccq"])
        raw_link_quality = "/".join([str(int(0.7 * link_quality)), "70"])
        health_report.link_quality_raw = raw_link_quality
        health_report.link_quality = link_quality / 100.0
        health_report.txpower = self._default_tx_power
        health_report.bitrate = solveBitrate(health_result)
        health_report.frequency = float(health_result["=channel"].split("/")[0]) / 1000.0
        self._connection_publisher.publish(health_report)
