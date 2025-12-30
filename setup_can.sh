#!/bin/bash

# 비트레이트 설정 (필요 시 수정)
BITRATE=1000000
DBITRATE=5000000  # CAN FD data phase bitrate

# 실행 모드 확인 (up 또는 down)
MODE=$1

if [ "$MODE" == "up" ]; then
    echo "Starting CAN FD interfaces (can0, can1)..."

    # can0 활성화 (CAN FD)
    sudo ip link set can0 down 2>/dev/null
    sudo ip link set can0 up type can bitrate $BITRATE dbitrate $DBITRATE fd on

    # can1 활성화 (CAN FD)
    sudo ip link set can1 down 2>/dev/null
    sudo ip link set can1 up type can bitrate $BITRATE dbitrate $DBITRATE fd on

    echo "CAN FD interfaces are UP (bitrate: $BITRATE bps, dbitrate: $DBITRATE bps)."
    ip link show can0
    ip link show can1

elif [ "$MODE" == "down" ]; then
    echo "Shutting down CAN interfaces (can0, can1)..."
    
    sudo ip link set can0 down
    sudo ip link set can1 down
    
    echo "CAN interfaces are DOWN."

else
    echo "Usage: ./setup_can.sh [up|down]"
fi
