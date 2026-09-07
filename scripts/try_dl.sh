#!/bin/bash
# Try downloading FAST-LIVO2 dataset bags from Google Drive, smallest-first, skip quota-blocked files.
declare -A FILES=(
 [HKU_Lecture_Center_02]=1aTc3db6T1zoUF3GNZ0CaaZvrv5ZufByx
 [HKU_Lecture_Center_01]=1W3ex0-180l0GhXXf3rwEvUN74NzplSS1
 [HKU_Centennial_Garden_02]=1innScHcQvQsTBsXHyPPgvIg1S4aSV_X5
 [HKU_Main_Building]=1eyOdK9yteK3LUkxaM3UAxg85vukq0B51
 [HKU_Landmark]=194gEIlkvrUsExoQlJXbq4vM6v1pvuJSP
 [HKU_Centennial_Garden_01]=15X21HhXfRxEvAq-KPUwFQvnJEo-7-58B
 [SYSU_01]=1R1y-T-0toyl38K4ZVQ6KW9qmxNWzjaZZ
)
ORDER="HKU_Lecture_Center_02 HKU_Lecture_Center_01 HKU_Centennial_Garden_02 HKU_Main_Building HKU_Landmark HKU_Centennial_Garden_01 SYSU_01"
mkdir -p /root/fast_livo2_data/hku
cd /root/fast_livo2_data/hku || exit 1
cp -n /root/fast_livo2_data/bright_screen_wall/calibration.yaml .
for NAME in $ORDER; do
  ID=${FILES[$NAME]}
  echo "== Trying $NAME ($ID)"
  PAGE=$(curl -sL -c /tmp/ck_$NAME "https://drive.usercontent.google.com/download?id=$ID&export=download")
  UUID=$(echo "$PAGE" | grep -oE 'name="uuid" value="[^"]*"' | cut -d'"' -f4)
  if [ -z "$UUID" ]; then
    echo "no confirm form; page hint: $(echo "$PAGE" | grep -oE 'Quota exceeded|Too many users' | head -1)"
    continue
  fi
  curl -sL -b /tmp/ck_$NAME -r 0-15 "https://drive.usercontent.google.com/download?id=$ID&export=download&confirm=t&uuid=$UUID" -o /tmp/probe_$NAME
  if ! head -c 8 /tmp/probe_$NAME | grep -q ROSBAG; then
    echo "BLOCKED: $(grep -oE 'Quota exceeded|Too many users' /tmp/probe_$NAME | head -1)"
    rm -f /tmp/probe_$NAME
    continue
  fi
  echo "OK, downloading $NAME.bag ..."
  curl -sL -b /tmp/ck_$NAME "https://drive.usercontent.google.com/download?id=$ID&export=download&confirm=t&uuid=$UUID" -o $NAME.bag
  SZ=$(stat -c%s $NAME.bag)
  echo "size=$SZ"
  if head -c 8 $NAME.bag | grep -q ROSBAG; then
    echo "VERIFIED $NAME.bag"
    exit 0
  fi
  echo "download failed, trying next"
  rm -f $NAME.bag
done
echo ALL_FAILED
