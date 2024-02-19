.text
init:
    # INIT interrupts
    la t0, ISR                  # load ISR address into t0
    csrrw zero, mtvec, t0       # set mtvec to ISR address
    li a7, 0                    # clear intr flag
    # INIT TC
    li t6, 0x1100D000           # TC CSR port address
    li t5, 0x1100D004           # TC count port address
    li t0, 50000000             # blink rate, 1Hz
    sw t0, 0(t5)                # init TC count
    li t0, 0x01                 # init TC CSR
    sw t0, 0(t6)                # no prescale, turn on TC

    li s10, 0x1100C000          # addr for leds
    li a0, 0                    # LED state

unmask:
    li t0, 0x8                  # enable interrupts
    csrrs zero, mstatus, t0     # enable interrupts

wait:
    beq a7, zero, wait          # wait for interrupt

blink:
    xori a0, a0,  1             # toggle LED
    sw a0, 0(s10)               # output LED    

    li a7, 0                    # clear interrupt flag
    j unmask                    # unmask interrupts

#--------------------------------------------------------------
# Interrupt Service Routine
#  desc: 
#--------------------------------------------------------------
ISR:
    li a7, 1                    # set interrupt flag
    li t2, 0x80                 # clear MPIE
    csrrc zero, mstatus, t2     # clear MPIE
    mret                        # return from interrupt

