#!/bin/sh

. $(dirname $(readlink -f $0))/boards.sh
uuids="" # Force IDs use by clearing UUIDs

cmd_base="./cli/vescpp_cli -P $can_port"

if [ "$ids" != "" ]; then 
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
      cmd="$cmd_base -u $uuid -f load_conf $cnf $cnff"
      echo $cmd
      $cmd;
    done
  done
elif [ "$uuids" != "" ]; then
  echo "Not implemented, use IDs"
else
  echo "IDs or UUIDs please !"
fi


