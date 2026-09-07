#!/bin/bash
# Patient chunked Google Drive downloader with resume.
# Quota is a per-IP token bucket; retry patiently. Small chunks (<~4MB) pass.
# Usage: chunked_dl.sh <file_id> <output.bag> [chunk_mb]
ID=$1
OUT=$2
CHUNK_MB=${3:-1}
CHUNK=$((CHUNK_MB*1024*1024))
URL="https://drive.usercontent.google.com/download?id=$ID&export=download"
LOG=${4:-/tmp/chunked_dl2.log}

log() { echo "$(date +%H:%M:%S) $*" >> "$LOG"; }

get_session() {
  PAGE=$(curl -sL -c /tmp/ck_dl "https://drive.usercontent.google.com/download?id=$ID&export=download")
  UUID=$(echo "$PAGE" | grep -oE 'name="uuid" value="[^"]*"' | cut -d'"' -f4)
  [ -n "$UUID" ] && DL="$URL&confirm=t&uuid=$UUID"
}

# Resume support
START=0
if [ -f "$OUT" ]; then
  START=$(stat -c%s "$OUT")
  log "resuming from $START"
fi

get_session
# Probe with retries until total size is known
TOTAL=""
for try in $(seq 1 240); do
  curl -s -b /tmp/ck_dl -r 0-$((CHUNK-1)) -D /tmp/h0 -o /tmp/chunk0 "$DL"
  TOTAL=$(tr -d '\r' < /tmp/h0 | grep -iE '^content-range' | grep -oE '[0-9]+$')
  [ -n "$TOTAL" ] && break
  log "probe retry $try"
  sleep 60
  get_session
done
SZ0=$(stat -c%s /tmp/chunk0 2>/dev/null || echo 0)
if [ -n "$TOTAL" ] && [ "$SZ0" -eq "$CHUNK" ]; then
  if [ "$START" -eq 0 ]; then cp /tmp/chunk0 "$OUT"; START=$CHUNK; fi
fi
if [ -z "$TOTAL" ]; then
  echo "ERROR: cannot determine total size"
  exit 1
fi
log "total $TOTAL, start $START"

while [ $START -lt $TOTAL ]; do
  END=$((START+CHUNK-1))
  [ $END -ge $TOTAL ] && END=$((TOTAL-1))
  EXPECT=$((END-START+1))
  OK=0
  for try in $(seq 1 200); do
    curl -s -b /tmp/ck_dl -r $START-$END -o /tmp/chunk_x "$DL"
    SZ=$(stat -c%s /tmp/chunk_x 2>/dev/null || echo 0)
    if [ "$SZ" -eq "$EXPECT" ] && ! head -c 15 /tmp/chunk_x | grep -q "<!DOCTYPE"; then
      cat /tmp/chunk_x >> "$OUT"
      OK=1
      break
    fi
    if [ $((try % 10)) -eq 1 ]; then log "chunk $START retry $try"; fi
    sleep 20
    get_session
  done
  [ $OK -eq 0 ] && { log "FAILED at offset $START"; exit 1; }
  START=$((END+1))
  if [ $((START % (CHUNK*20) )) -lt $CHUNK ]; then log "progress $((START*100/TOTAL))% ($START/$TOTAL)"; fi
done
GOT=$(stat -c%s "$OUT")
log "final size: $GOT / $TOTAL"
if [ "$GOT" -eq "$TOTAL" ] && head -c 8 "$OUT" | grep -q ROSBAG; then
  log "VERIFIED"
  echo VERIFIED
else
  log "VERIFY FAILED"
  exit 1
fi
