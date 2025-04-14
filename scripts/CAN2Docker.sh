#!/bin/sh

if [ $# -lt 2 ]; then
  echo "Usage: $0 <docker_ct> <can_port>"
  exit 0
fi
docker_ct="$1"
can="$2"
lcan=vx$can
rcan=vxcan1

DOCKERPID=$(docker inspect -f '{{ .State.Pid }}' $docker_ct)
modprobe can-gw
ip link add $lcan type vxcan peer name $rcan netns $DOCKERPID
cangw -A -s $can  -d $lcan -e
cangw -A -s $lcan -d $can  -e
ip link set $lcan up
#ip link set $can type can bitrate 1000000
#ip link set $can up
nsenter -t $DOCKERPID -n ip link set $rcan up