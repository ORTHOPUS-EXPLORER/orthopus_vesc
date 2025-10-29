#!/bin/sh

. $(dirname $(readlink -f $0))/boards.sh
ids="" # Force UUIDs use by clearing IDs

cmd_base="./cli/vescpp_cli -P $can_port"

if [ "$ids" != "" ]; then 
  for i in $ids; do 
    cmd="$cmd_base -i $i -f flash_fw app $fw_file"
    echo $cmd
    $cmd;
  done
elif [ "$uuids" != "" ]; then
  for u in $uuids; do 
    cmd="$cmd_base -u $u -f flash_fw app $fw_file"
    echo $cmd
    $cmd;
  done
else
  echo "IDs or UUIDs please !"
fi
