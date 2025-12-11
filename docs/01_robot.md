1&emsp; About the Turtlebot3 Burger
=====

The Turtlebot3 burger used in this workshop is modifed to have the following hardware:
- Raspberry Pi 5, 8GB
- Raspberry Pi Camera V3
- Rplidar C1

A PC is used to connect to the robot. Most of the computation occurs on the robot, and the PC is used to interface with the robot. 

# 1&emsp;Robot Username, Team Number and ROS_DOMAIN_ID

If the team number is 3, then take note of the following values. Please keep in mind that the robot username is in double digit form.

<table><tbody>
  <tr><td><b>Team number</b></td><td>3</td></tr>
  <tr><td><b>Robot username</b></td><td><code>ece03</code></td>
  <tr><td><code>ROS_DOMAIN_ID</code></td><td><code>3</code></td>
</tbody></table>

# 2&emsp;Robot Power Modes
The following table and image describe the modes of powering the turtlebot. 
Always switch to **tether mode** if the robot needs to be on standby while you troublehsoot, as tethering prevents the battery from discharging too quickly.

| Mode | Description |
|-|-|
| **Tether** | Connect the power adapter to the DC jack on the OpenCR board. |
| **Battery** | Connect the battery and disconnect the power adapter from the OpenCR board. |

![]()


# 3&emsp;Monitor and Keyboard
The robot can be connected to the monitor and keyboard. 
Connecting the monitor and keyboard is essential for finding out the IP address of the robot or re-connecting the robot to another Wi-Fi.

| Mode | Description |
|-|-|
| **With Keyboard and Monitor** | Connect the micro-HDMI cable, that is attached to the monitor, to the Raspberry Pi. Then connect the USB cable of the keyboard to the Raspberry Pi. |
| **Without Keyboard and Monitor** | Simply disconnect both the monitor and the keyboard. |


# 4&emsp;Booting up the Robot
1. Power the robot by using either the *tether* mode (advised) or the *battery* mode.
2. Connect the robot to its peripherals (monitor and keyboard), if the IP address needs to be obtained, or if another Wi-Fi is to be connected. Otherwise, the peripherals need not be connected.
3. Flick the switch on the OpenCR board to the right.
4. Wait for a melody to play from the OpenCR board, and then wait for about 15 to 20 seconds for the robot to start up. The raspberry Pi should have a non-blinking green light when it is ready.

# 5&emsp;Obtain the IP Address
1. Ensure that the robot is powered up and connected to the peripherals (monitor and keyboard).
2. Login to the robot if not already:
    1. On the keyboard connected to the robot, type the username of the robot, and press `ENTER`.
    2. Type the password. The password will not show as it is typed. Press `ENTER` when done.
    3. Try again if it fails.
3. Once the control is given back to you, type the following and run (by pressing `ENTER`).
    ```bash
    ip a
    ```
    The command is short for `ip all`.
4. If the robot is connected to a Wi-Fi, the following text may be seen:
    ```bash
    3: wlan0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc noqueue state UP group default qlen 1000
        link/ether 2c:11:aa:2c:ee:f7 brd ff:ff:ff:ff:ff:ff
        inet 192.168.1.18/24 brd 192.168.1.255 scope global dynamic noprefixroute wlan0
            valid_lft 252sec preferred_lft 252sec
        inet6 fe80::25ea:6c25:730c:b707/64 scope link noprefixroute 
            valid_lft forever preferred_lft forever
    ```
    Look for `wlan0`, and the IP address is next to the `inet` in the third line of the example above. The IP address **does not include** `/24`. In the example, the IP address is `192.168.1.18`.

# 6&emsp;Changing the Wi-Fi connection
1. Ensure that the robot is powered up and connected to the peripherals (monitor and keyboard).
2. Login to the robot if not already.
3. Run the following command to edit the network configuration file:
    ```bash
    sudo nano /etc/netplan/50-cloud-init.yaml
    ```
    Hint: You can simply type `sudo nano /etc/netplan/5` and press `TAB` to auto-complete.
4. Key in the password if prompted.

5. A YAML file will open, which may look like the following:

    ```yaml
    network:
      version: 2
      ethernets:
        eth0:
          optional: true
          dhcp4: true
      wifis:
        wlan0:
          optional: true
          dhcp4: true
          access-points:
            "a_wifi_ssid":
              password: "the_wifi_password"
    ```
6. Suppose the Wi-Fi or hotspot to be connected is called `nusece` and the password is `applying`. Modify the last two lines between the quotes `""`. Do not modify anything else.

    ```yaml
    network:
      version: 2
      ethernets:
        eth0:
          optional: true
          dhcp4: true
      wifis:
        wlan0:
          optional: true
          dhcp4: true
          access-points:
            "nusece":
              password: "applying"
    ```

7. Press `Ctrl+S` and `Ctrl+X` to save and exit.

8. Run the following command:
    ```bash
    sudo netplan apply
    ```

9. Run the following command repeatedly until the line containing `wlan0` shows two green words `routable` and `configured`:
    ```bash
    networkctl
    ```

10. The new IP address can subsequently be found by running:
    ```bash
    ip a
    ```

# 7&emsp;Turning off the Robot
1. In a terminal of the robot (the robot's username can be seen on the bottom-left of the terminal), run the following command:
  ```bash
  sudo shutdown now
  ```
2. Wait for about 5 seconds, and until the Raspberry Pi's LED has turned red and stopped blinking.
3. Flick the switch on the OpenCR board to the left.