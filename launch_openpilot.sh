#!/usr/bin/bash

./stressdisk cycle /data/tmp/ &

export BLOCK=uploader
export FINGERPRINT="TOYOTA COROLLA TSS2 2019"
export PASSIVE="0"
exec ./launch_chffrplus.sh
