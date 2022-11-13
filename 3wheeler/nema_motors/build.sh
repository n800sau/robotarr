#!/bin/bash

platformio run -e stm32 &> build.log
#platformio run
echo $?
