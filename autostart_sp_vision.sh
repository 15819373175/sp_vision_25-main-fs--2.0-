#!/bin/bash

# SP Vision Auto Aim Autostart Script
# 此脚本用于桌面环境自动启动 SP Vision 自动瞄准程序

WORK_DIR=/home/robomaster/sp_vision_25-main
EXEC_FILE=$WORK_DIR/build/auto_aim_debug_mpc
CONFIG_FILE=configs/standard9.yaml

cd $WORK_DIR

# 检查可执行文件是否存在
if [ ! -f $EXEC_FILE ]; then
    echo "Error: Executable not found: $EXEC_FILE"
    exit 1
fi

# 启动程序
echo "Starting SP Vision Auto Aim..."
$EXEC_FILE $CONFIG_FILE

# 如果程序退出，保持终端打开
exec bash
