set remotetimeout 2000
target extended-remote localhost:3333
load
monitor reset init
monitor reset run
detach
quit
