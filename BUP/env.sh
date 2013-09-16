#!/bin/sh

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

if [[ "x$LUA_PATH" == "x" ]]; then LUA_PATH=";;"; fi

if [[ ! -d $DIR ]]; then
    echo "ERROR: failed to detect path to microblx"
fi

UBX_LUA_PATH="$DIR/lua/?.lua"

export LUA_PATH="$LUA_PATH;$UBX_LUA_PATH"
export LD_LIBRARY_PATH=/home/evert/h5fddsm/h5fddsm-0.9.9/build/bin/:/home/evert/local/lib
