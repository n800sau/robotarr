set remotetimeout 2000
target extended-remote :3333
monitor arm semihosting enable
load
monitor reset init
