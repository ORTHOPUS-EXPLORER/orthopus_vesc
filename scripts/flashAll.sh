#!/bin/sh

can_port=""
dir=""
ids=""
uuids=""

if [ -z "$1" -o "$1" = "bench" ]; then
  echo "Config: Bench"
  can_port="can0"
  dir="bench"
  if [ -z "$2" -o "$2" = "bmi_edu" ]; then
    uuids="$uuids 0x280022000547323132343933"  # 11
    uuids="$uuids 0x2c0040000547323132343933"  # 12
    uuids="$uuids 0x59002c0013504b4d31383120"  # 13
    uuids="$uuids 0x4e00260013504b4d31383120"  # 14
    uuids="$uuids 0x310036000547323132343933"  # 15
    uuids="$uuids 0x1f00330014504b4d31383120"  # 16
    fw_file="../edu.bin"
  elif [ -z "$2" -o "$2" = "bmi_a50s" ]; then
    can_port="can3"
    uuids="$uuids 0x2d002f001247333034333237"  # 11
    uuids="$uuids 0x2d0040001247333034333237"  # 12 
    uuids="$uuids 0x250022001147333034333237"  # 13
    fw_file="../a50s_v23c_8s.bin"
  else
    ids=`seq 11 16`
  fi
elif [ "$1" = "explorer" ]; then
  echo "Config: Explorer"
  can_port="can2"
  dir="explorer"
  ids=`seq 11 16`
  fw_file="../edu.bin"
else
  echo "Usage: $0 [explorer|bench]. Abort"
  exit 0
fi

cmd_base="./cli/vescpp_cli -f -P $can_port"

if [ "$ids" != "" ]; then 
  for i in $ids; do 
    cmd="$cmd_base -i $i flash_fw app $fw_file"
    echo $cmd
    $cmd;
  done
elif [ "$uuids" != "" ]; then
  for u in $uuids; do 
    cmd="$cmd_base -u $u flash_fw app $fw_file"
    echo $cmd
    $cmd;
  done
else
  echo "IDs or UUIDs please !"
fi
