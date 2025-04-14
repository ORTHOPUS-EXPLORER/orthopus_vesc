#!/bin/sh

can_port=""
dir=""
ids=""
if [ -z "$1" -o "$1" = "bench" ]; then
  echo "Config: Bench"
  can_port="can0"
  dir="bench"
  ids=`seq 11 16`
elif [ "$1" = "explorer" ]; then
  echo "Config: Explorer"
  can_port="can2"
  dir="explorer"
  ids=`seq 11 16`
else
  echo "Usage: $0 [explorer|bench]. Abort"
  exit 0
fi

cmd_base="./cli/vescpp_cli -f -P $can_port"

for i in $ids; do 
  for cnf in "motor" "custom" "app"; do
    cnff="`ls -Art $dir/${cnf}_conf_${i}_*.json 2> /dev/null | tail -n 1`"
    if [ -z "$cnff" ]; then
      continue
    fi
    uuidb=`cat $cnff | jq -r '.["info"]["uuid"]["bytes"] | .[]'`
    uuid="0x"
    for ub in $uuidb; do
      uuid="${uuid}`printf "%02x" $ub`"
    done
    cmd="$cmd_base -i $i -u $uuid load_conf $cnf $cnff"
    #echo $cmd
    $cmd;
  done
done
