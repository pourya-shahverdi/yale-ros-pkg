# **THIS PAGE IS DEPRECATED! #
## Please go to http://robotics.usc.edu/git/sar-expeditions/pages/DragonbotPubSubInstall for the new home of the dragonbot drivers ##**

# Introduction #

Currently code for this project is contained in two repositories:

  * http://code.google.com/r/davidfeilseifer-dragonbot-mod/
  * http://code.google.com/p/yale-ros-pkg/

The checkout of the repositories can be automated. However, the installation of the code takes a little more doing. Apologies for the challenge.

# To Download the Repositories #

  1. install ROS groovy: http://www.ros.org/wiki/groovy/Installation/Ubuntu
  1. install java: sudo apt-get install openjdk-7-jdk
  1. install rosws: http://www.ros.org/doc/independent/api/rosinstall/html/
  1. checkout the respositories into a workspace `<ws_name>`:
```
> rosws init ~/<ws_name> /opt/ros/groovy
> source ~/<ws_name>/setup.bash
> rosws merge http://robotics.usc.edu/~dfseifer/dragonbot_actionlib.rosinstall
> source ~/<ws_name>/setup.bash
> rosws update
```
  1. `> rosmake dragonbot_relay`
  1. install rosjava _(Note that **org.ros.time.NtpTimeProviderTest** might fail on the build step.  This is fine and you can proceed to the next step)_:
```
> roscd rosjava_core
> ./gradlew build
> ./gradlew install
```
  1. install support packages
```
> roscd dragonbot_jars
> ./gradlew build install
```
  1. install dragonbot\_pubsub:
```
> roscd dragonbot_pubsub
> ./gradlew build install installApp
```

# Network Configuration #

After the installation, when you want to connect to the phone, you need to change the network interface to en1... Here's how:

  1. let's say the the phone is on a network which your interface eth1 connects to, use ifconfig to see which interface is active
  1. edit /etc/udev/rules.d/70-persistent-net.rules:
    * `PCI device 0x1011:/sys/devices/pci0000:00/0000:00:1c.5/0000:04:00.0/0000:05:01.0/0000:06:07.0 (tulip) SUBSYSTEM=="net", ACTION=="add", DRIVERS=="?*", ATTR{address}=="00:60:f5:08:7b:27", ATTR{dev_id}=="0x0", ATTR{type}=="1", KERNEL=="eth*", NAME="eth3"`
    * becomes
    * `PCI device 0x1011:/sys/devices/pci0000:00/0000:00:1c.5/0000:04:00.0/0000:05:01.0/0000:06:07.0 (tulip) SUBSYSTEM=="net", ACTION=="add", DRIVERS=="?*", ATTR{address}=="00:60:f5:08:7b:27", ATTR{dev_id}=="0x0", ATTR{type}=="1", KERNEL=="eth*", NAME="en1"`
  1. then reboot
# Running Dragonbot #
> Dragonbot can then be run with the following command:

```
> roslaunch dragonbot_relay dragonbot_relay.launch
```

and in a different terminal

```
> roscd dragonbot_pubsub
> build/install/dragonbot_pubsub/bin/dragonbot_pubsub edu.yale.dragonbot.DragonbotNode
```