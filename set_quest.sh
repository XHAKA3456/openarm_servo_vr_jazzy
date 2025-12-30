#!/bin/bash

# ADB 포트 포워딩 설정
echo "Setting up ADB reverse ports..."

adb reverse tcp:5454 tcp:5454
adb reverse tcp:5656 tcp:5656

echo "ADB reverse completed: 5454, 5656"
