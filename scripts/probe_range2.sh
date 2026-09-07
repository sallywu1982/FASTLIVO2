#!/bin/bash
ID=1aTc3db6T1zoUF3GNZ0CaaZvrv5ZufByx
URL="https://drive.usercontent.google.com/download?id=$ID&export=download"
PAGE=$(curl -sL -c /tmp/ck_pr2 "https://drive.usercontent.google.com/download?id=$ID&export=download")
UUID=$(echo "$PAGE" | grep -oE 'name="uuid" value="[^"]*"' | cut -d'"' -f4)
echo "uuid=$UUID"
for R in "0-1048575" "1048576-2097151" "2097152-3145727" "1048576-5242879"; do
  curl -s -b /tmp/ck_pr2 -r $R -o /tmp/pr_out2 "$URL&confirm=t&uuid=$UUID"
  SZ=$(stat -c%s /tmp/pr_out2)
  MAGIC=$(head -c 8 /tmp/pr_out2 | tr -d '\0')
  echo "range $R -> got $SZ bytes, starts: ${MAGIC:0:15}"
done
