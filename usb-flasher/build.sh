#!/bin/bash

set -e

gcc -o ppkb-flasher flasher.c
gcc -o ppkb-debugger debugger.c
