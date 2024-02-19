init:
    li s0, 0x1100C000           # addr for leds
    
    la t0, ISR                  # load ISR address into t0
    csrrw zero, mtvec, t0       # set mtvec to ISR address
    li a7, 0                    # clear interrupt flag

    li a0, 0                    # led state

unmask:
    li t0, 0x8                  # enable interrupts
    csrrs zero, mstatus, t0     # enable interrupts
wait:
    beq a7, zero, wait          # wait for interrupt

blink:
    xori a0, a0,  1             # toggle LED
    sw a0, 0(s0)                # output LED    

    li a7, 0                    # clear interrupt flag
    j unmask                    # unmask interrupts


#--------------------------------------------------------------
# Interrupt Service Routine
#--------------------------------------------------------------
ISR:
    li a7, 1                    # set interrupt flag
    li t2, 0x80                 # clear MPIE
    csrrc zero, mstatus, t2     # clear MPIE
    mret                        # return from interrupt