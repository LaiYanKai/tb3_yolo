#!/bin/bash
set -exo pipefail

CONN_NAME="Hotspot"
SSID="ssid"
PASS="password"
WIFI_IFACE="wlan0" # See `ip a` for interface
BAND="a" # 5Ghz
CHANNEL=36

# To see the available channels, run the following and look for the number in [], e.g. 5180.0 MHz [36] (23.0 dBm)
# iw phy phy0 info
# 36,40,44,48,
# 52,60,64,100,104,108,112,116,120,124,128,132,136,140,144, # (radar detection)
# 149,153,157,161,165,

# # The following to set up 5Ghz AP, from https://www.reddit.com/r/Ubuntu/comments/1fvaasw/force_5ghz_wifi_hotspot_ubuntu_2404/
#
# # disable randomization
# sudo nano /etc/NetworkManager/conf.d/90-disable-randomization.conf
# # # type:
# # [device-mac-randomization]
# # wifi.scan-rand-mac-address=no
#
# # install iw for setting 5GHz requirements
# sudo apt install iw
#
# # set to SG requirements
# sudo iw reg set SG
#
# # make persistent
# sudo nano /etc/default/crda
# # # type: 
# # REGDOMAIN=SG

# Delete existing hotspot profile if it exists
sudo nmcli con down "$CONN_NAME" || echo "'$CONN_NAME' is already inactive. Ignore last two errors."
sudo nmcli con delete "$CONN_NAME" || echo "'$CONN_NAME' is not active. Ignore previous error."

sudo nmcli con add \
  type wifi \
  ifname "$WIFI_IFACE" \
  con-name "$CONN_NAME" \
  autoconnect yes \
  ssid "$SSID"
  
# band: a(5GHz) or bg(2.4GHz)
sudo nmcli con mod "$CONN_NAME" \
  802-11-wireless.mode ap \
  ipv4.method shared \
  wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PASS" \
  802-11-wireless.band $BAND \
  802-11-wireless.channel $CHANNEL

# Restart connection to apply
sudo systemctl restart NetworkManager
nmcli con up "$CONN_NAME"