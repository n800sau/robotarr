#!/usr/bin/env python

import time
from reset import reset_stm32
from serprot import find_hw

reset_stm32()
time.sleep(3)
dev = find_hw()
if dev:
	print('found', dev)
else:
	print('not found')
