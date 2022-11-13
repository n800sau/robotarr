source vars.sh

#~/work/roborep/bin/miniterm.py -p $DEV -b 115200
#../../bin/miniterm.py -p $DEV -b 9600 --rts 0 --dtr 0 &>runterm.log
#miniterm.py -p $DEV -b 115200 --rts 0 --dtr 0 &>runterm.log
#platformio device monitor --port $DEV -b 115200 -f esp32_exception_decoder -f time &>runterm.log
platformio device monitor --port $DEV -b 115200 -f time --rts 0 --dtr 0 &>runterm.log
#platformio device monitor --port $DEV -b 115200 -f time --rts 0 --dtr 0
#platformio device monitor --rts 0 --dtr 0 --port $DEV -b 115200 -f esp32_exception_decoder -f time &>runterm.log
#../../bin/miniterm.py -p $DEV -b 74880 --rts 0 --dtr 0

echo $?
