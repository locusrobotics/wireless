import argparse
import rospy
from std_msgs.msg import Bool, Header
from wireless_watcher.mikrotik_api import RouterOSApi
from wireless_msgs.msg import Connection
import subprocess

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
        return self.silentGet(["/interface/wireless/monitor","=numbers={}".format(interface),"=once="])

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
        health_report.link_quality = link_quality
        health_report.txpower = self._default_tx_power
        health_report.bitrate = solveBitrate(health_result)
        health_report.frequency = float(health_result["=channel"].split("/")[0])
        self._connection_publisher.publish(health_report)
