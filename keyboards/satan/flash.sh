#!/usr/bin/env bash

PATH="/usr/local/CrossPack-AVR/bin:$PATH"
make clean
make KEYMAP=frankchang dfu
