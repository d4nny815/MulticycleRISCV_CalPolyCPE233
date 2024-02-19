#====================
# Routine
#   Move an led from right to left after a certain delay then reset led
#
#====================
init:
    li s0, 0x1100C000   # led addr
    li a0, 1            # cur led val
    li a1, 0x8000       # max led pos
    sw a0, 0(s0)        # store led val to leds
main:
    slli a0, a0, 1      # move led pos to left
    sw a0, 0(s0)        # save to leds
    call delay          # delay
    bne a1, a0, main    # jump back to main if not at max 
    li a0, 1            # reset led pos
    j main              # go back 

delay:
    li t0, 0x2FFFF      # load delay value
delay_loop:
    addi t0, t0, -1     # decrement delay
    bne zero, t0, delay_loop
    ret
