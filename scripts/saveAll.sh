#!/bin/sh

. $(dirname $(readlink -f $0))/boards.sh
ids="" # Force UUIDs use by clearing IDs

cmd_base="./cli/vescpp_cli -P $can_port"

if [ "$ids" != "" ]; then 
  for i in $ids; do 
    for cnf in "app" "motor" "custom"; do
      cmd="$cmd_base -i $i -D $dir save_conf $cnf"
      echo $cmd
      $cmd;
    done
  done
elif [ "$uuids" != "" ]; then
  for u in $uuids; do 
    for cnf in "app" "motor" "custom"; do
      cmd="$cmd_base -u $u -D $dir save_conf $cnf"
      echo $cmd
      $cmd;
    done
  done
else
  echo "IDs or UUIDs please !"
fi
