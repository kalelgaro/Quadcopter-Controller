#!/bin/bash

make
st-flash --reset write ./quad.bin 0x08000000
openocd -f stm32f4discovery.cfg
