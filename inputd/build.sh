#!/bin/sh

php map-to-c.php factory-keymap.txt > kmap.h
gcc -o ppkbd main.c || exit 1
