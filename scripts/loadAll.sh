#!/bin/sh

can_port=""
dir=""
ids=""
if [ -z "$1" -o "$1" = "bench" ]; then
  echo "Config: Bench"
  can_port="can0"
  ids=`seq 11 16`
  dir="bench"
  if [ -z "$2" -o "$2" = "bmi_edu" ]; then
    dir="$dir/edu/"
  elif [ -z "$2" -o "$2" = "bmi_a50s" ]; then
    can_port="can3"
    dir="$dir/a50s/"
  fi
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
      echo "id $i conf $cnf not found in $dir"
      continue
    fi
    uuidb=`cat $cnff | jq -r '.["info"]["uuid"]["bytes"] | .[]'`
    uuid="0x"
    for ub in $uuidb; do
      uuid="${uuid}`printf "%02x" $ub`"
    done
    cmd="$cmd_base -u $uuid load_conf $cnf $cnff"
    echo $cmd
    $cmd;
  done
done
