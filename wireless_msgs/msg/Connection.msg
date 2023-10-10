std_msgs/Header header

float32 bitrate  # Mb/s
int16 txpower  # dBm

# Hardware/Firmware
string wifi_chip
string firmware_version
string kernel_version

# Fractional link quality number preserved in raw
# field, resolved to decimal for link_quality field.
string link_quality_raw
float32 link_quality

int16 signal_level  # dBm
int16 noise_level  # dBm

string essid
string bssid
float64 frequency

string ipv4 # dot notation ipv4 for this connection. For robots this is typically the wlan0 IP address.
