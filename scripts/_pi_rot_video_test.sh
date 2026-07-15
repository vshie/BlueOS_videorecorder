#!/bin/bash
# Record an RTSP clip on the Pi while driving a rotation-counted move via the
# extension API, so we can measure physical disc revolutions (wedge angle in
# video) against the sensor's counted edges.
#
# Usage: _pi_rot_video_test.sh <direction> <rotations> <record_seconds> <tag>
set -u
DIR="${1:-unwind}"
ROT="${2:-3}"
RECS="${3:-14}"
TAG="${4:-run}"
IMG=vshie/blueos-blueos_video_recorder:dropcam
URL=rtsp://127.0.0.1:8554/video_stream__dev_video2
OUT=/home/pi/rotpoll
API=http://127.0.0.1:5423
mkdir -p "$OUT"
docker rm -f rot-rec >/dev/null 2>&1

jq_num() { python3 -c "import sys,json;print(json.load(sys.stdin).get('$1'))"; }

# Start recording (background container, copy codec).
docker run -d --name rot-rec --network host --entrypoint ffmpeg \
  -v "$OUT":/out "$IMG" -y -loglevel error -rtsp_transport tcp \
  -i "$URL" -t "$RECS" -c copy -f matroska "/out/move_${TAG}.mkv" >/dev/null 2>&1

sleep 2.0
BEFORE=$(curl -s -m6 "$API/telemetry" | jq_num rotation_count)
TREL=$(python3 -c "import time;print('%.2f'%time.monotonic())")
curl -s -m6 -X POST "$API/release" -H 'Content-Type: application/json' \
  -d "{\"action\":\"rotate\",\"direction\":\"$DIR\",\"rotations\":$ROT}" >/dev/null
echo "MOVE_START_REL=$TREL  (recording began ~2.0s earlier)"

# Poll until the release run finishes (covers canonical wind-finish too).
RUNNING=True
for i in $(seq 1 300); do
  RUNNING=$(curl -s -m6 "$API/release" | jq_num running)
  [ "$RUNNING" = "False" ] && break
  sleep 0.1
done
sleep 0.6
AFTER=$(curl -s -m6 "$API/telemetry" | jq_num rotation_count)
echo "BEFORE=$BEFORE AFTER=$AFTER GLOBAL_DELTA=$((AFTER-BEFORE)) dir=$DIR target=$ROT"

# Report the structured last-run result (delivered/outcome/canonical).
curl -s -m6 "$API/status" | python3 -c "
import sys,json
try:
    d=json.load(sys.stdin)
    r=d.get('last_release_rotation_result') or d.get('release_last_result')
    print('LAST_RESULT=',json.dumps(r))
except Exception as e:
    print('LAST_RESULT_ERR',e)
" 2>/dev/null

docker wait rot-rec >/dev/null 2>&1
docker rm -f rot-rec >/dev/null 2>&1
ls -la "$OUT/move_${TAG}.mkv"
