#!/bin/bash
MY_PATH=$(dirname "$0")
UNIX_EPOCH_TIME=$(date +%s) # %s = seconds since the Epoch (1970-01-01 00:00 UTC)
$MY_PATH/px4_shell_command.py -p "system_time set $UNIX_EPOCH_TIME"