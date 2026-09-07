#!/bin/bash
ID=1aTc3db6T1zoUF3GNZ0CaaZvrv5ZufByx
URL="https://drive.usercontent.google.com/download?id=$ID&export=download"
PAGE=$(curl -sL -c /tmp/ck_pr "https://drive.usercontent.google.com/download?id=$ID&export=download")
UUID=$(echo "$PAGE" | grep -oE 'name="uuid" value="[^"]*"' | cut -d'"' -f4)
echo "uuid=$UUID"
for END in 15 65535 262143 1048575 4194303 8388607; do
  curl -s -b /tmp/ck_pr -r 0-$END -o /tmp/pr_out "$URL&confirm=t&uuid=$UUID"
  SZ=$(stat -c%s /tmp/pr_out)
  MAGIC=$(head -c 8 /tmp/pr_out | tr -d '\0')
  echo "range 0-$END -> got $SZ bytes, starts with: ${MAGIC:0:20}"
done
