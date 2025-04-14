#!/bin/sh

cmd_base="./cli/vescpp_cli -P can2 -D explorer"

for i in $(seq 11 16); do 
  for cnf in "app" "motor" "custom"; do
    cmd="$cmd_base -i $i save_conf $cnf"
    echo $cmd
    $cmd;
  done
done
