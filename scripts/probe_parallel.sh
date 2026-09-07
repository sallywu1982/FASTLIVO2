#!/bin/bash
# Test whether quota bucket is per-session: N parallel fresh sessions each fetch a distinct 1MB range.
ID=$1
N=${2:-4}
for i in $(seq 1 $N); do
 (
  CJ=/tmp/pck_$i
  PAGE=$(curl -sL -c $CJ "https://drive.usercontent.google.com/download?id=$ID&export=download")
  U=$(echo "$PAGE" | grep -oE 'name="uuid" value="[^"]*"' | cut -d'"' -f4)
  S=$(( (i-1)*1048576 ))
  E=$(( i*1048576 - 1 ))
  curl -s -b $CJ -r $S-$E -o /tmp/pp_$i "https://drive.usercontent.google.com/download?id=$ID&export=download&confirm=t&uuid=$U"
  SZ=$(stat -c%s /tmp/pp_$i 2>/dev/null || echo 0)
  M=$(head -c 8 /tmp/pp_$i | tr -d '\0' | head -c 8)
  echo "session $i range $S-$E -> $SZ bytes (${M:0:8})"
 ) &
done
wait
