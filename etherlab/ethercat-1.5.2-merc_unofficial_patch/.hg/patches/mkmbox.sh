#!/bin/sh
if [ ! -f series ]; then
    echo "Run from the patches directory." >&2
    exit 1
fi
TARGET=${1:-all.mbx}
rm -f "$TARGET"
for p in `cat series`; do
    if [ ! -f $p ]; then continue; fi
    echo "Recording $p"
    echo "From hg-patch $(date -u '+%a %b %d %H:%M:%S %Y')" >> "$TARGET"
    echo "Message-ID: $p" >> "$TARGET"
    cat $p >> "$TARGET"
    echo "" >> "$TARGET"
done
echo "Combined patch mbox $TARGET created."
