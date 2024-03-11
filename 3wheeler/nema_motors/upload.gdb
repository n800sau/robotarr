set remotetimeout 4000
target extended-remote localhost:3333
load
monitor reset run
detach
quit
