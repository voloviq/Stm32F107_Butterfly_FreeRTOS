#target remote localhost:3333
#monitor reset
#monitor sleep 1500
#monitor poll
#monitor soft_reset_halt
#monitor gdb_breakpoint_override hard



cat .gdbinit
target remote localhost:3333
monitor reset
monitor sleep 500
monitor poll
monitor soft_reset_halt
#monitor arm7_9 force_hw_bkpts enable
monitor gdb_breakpoint_override hard
#monitor sleep 100
#monitor soft_reset_halt

