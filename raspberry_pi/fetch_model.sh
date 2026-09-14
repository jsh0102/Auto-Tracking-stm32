#!/usr/bin/env bash
# YOLOv4-tiny 가중치 내려받기 (24MB, 저장소에는 커밋하지 않음)
set -euo pipefail
cd "$(dirname "$0")"
URL=https://github.com/AlexeyAB/darknet/releases/download/yolov4/yolov4-tiny.weights
if [ -f yolov4-tiny.weights ]; then echo "이미 존재합니다."; exit 0; fi
echo "다운로드 중: $URL"
curl -fL -o yolov4-tiny.weights "$URL"
echo "완료: $(du -h yolov4-tiny.weights | cut -f1)"
