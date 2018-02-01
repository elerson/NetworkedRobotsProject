#!/bin/bash


for ((i=1;i<$(($2));i++))
do
  del_ip="192.168.0."$i
  sudo ip route del $del_ip
done

ip="192.168.0."$1
ifconfig wlan0 down
iwconfig wlan0 mode ad-hoc
iwconfig wlan0 channel 5
ifconfig wlan0 $ip/24 up
sudo sysctl -w net.ipv4.ip_forward=1
iptables --verbose --table filter --policy FORWARD ACCEPT
iptables --verbose --table nat --append POSTROUTING --jump MASQUERADE


iwconfig wlan0 essid router_robots
#ip route add 192.168.0.1 via 192.168.0.1 dev wlan0
#ip route add 192.168.0.3 via 192.168.0.3 dev wlan0

for ((i=1;i<$(($1));i++))
do
  low_ip="192.168.0."$i
  previous_ip="192.168.0."$(($1-1))
  ip route add $low_ip via $previous_ip dev wlan0
done

for ((i=$(($1+1));i<$(($2+1));i++))
do
  up_ip="192.168.0."$i
  next_ip="192.168.0."$(($1+1))
  ip route add $up_ip via $next_ip dev wlan0
done
