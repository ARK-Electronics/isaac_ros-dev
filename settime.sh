#!/bin/bash
HOST_IP=127.0.0.1
HOST_PORT=14551

MY_PATH=$(dirname "$0")
UNIX_EPOCH_TIME=$(date +%s) # %s = seconds since the Epoch (1970-01-01 00:00 UTC)
$MY_PATH/px4_shell_command.py -p "udp:$HOST_IP:$HOST_PORT" "system_time set $UNIX_EPOCH_TIME"
