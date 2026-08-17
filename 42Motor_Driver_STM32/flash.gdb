set confirm off
target extended-remote localhost:61234
monitor halt
monitor reset
load
monitor reset
quit
