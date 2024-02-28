source vars.sh

#~/.local/bin/esptool.py --before no_reset --after soft_reset --chip esp8266 --port "/dev/serial/by-id/usb-1a86_USB2.0-Ser_-if00-port0" --baud 115200 write_flash 0x0 .pio/build/esp01/firmware.bin
#~/.local/bin/esptool.py --before usb_reset --after soft_reset --chip esp8266 --port "/dev/serial/by-id/usb-1a86_USB2.0-Ser_-if00-port0" --baud 74880 write_flash 0x0 .pio/build/esp01/firmware.bin

#~/work/roborep/ino/set_stm32_mode/set_Pmode.py

#platformio run -v -t upload --upload-port $DEV

#./set_boot0_high_flash.py && \
#platformio run -e stm32 -t upload --upload-port $DEV &>upload.log
#platformio run -e stm32 -t upload &>upload.log
#platformio run -e stm32 -t upload --upload-port $DEV &>upload.log
#platformio run -e stm32 -t upload &>upload.log

./build.sh && \
gdb-multiarch -q --command=upload.gdb --args ".pio/build/stm32/firmware.elf"

#&& \
#./set_boot0_low_run.py
echo $?

#~/work/roborep/ino/set_stm32_mode/set_Wmode.py

