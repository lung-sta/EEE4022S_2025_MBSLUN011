#!/bin/bash

start=$(date +%s.%N)

# check for extenstions and arguements
if [ -z "$1" ]; then
  TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
  OUTPUT_FILE="photo_$TIMESTAMP.jpg"
  echo "No filename provided. Using: $OUTPUT_FILE"
else
  if [[ ! "$1" =~ \.jpg$ ]]; then
    OUTPUT_FILE="$1.jpg"
    echo "Warning: File extension missing. Using: $OUTPUT_FILE"
  else
    OUTPUT_FILE="$1"
  fi
fi

# run libcamera-still silently
libcamera-still \
  --awb auto \
  --exposure normal \
  --shutter 100000 \
  --gain 1.0 \
  --width 4096 --height 2160 \
  -o "$(pwd)/blank_2g_images/$OUTPUT_FILE" \
  --autofocus-mode manual \
  --lens-position 2.5 > /dev/null 2>&1

end=$(date +%s.%N)
duration=$(echo "$end - $start" | bc)


echo "Picture Taken: $OUTPUT_FILE - $(date)"

printf "takePic.sh script took: %.5fs\n" "$duration"
