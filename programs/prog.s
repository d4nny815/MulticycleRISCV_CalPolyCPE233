/*
Counts the number of interrupts received by the system; a push of the left-most button on
the development board should generate an interrupt. The count sequence should be the
following: {0,1,2,3... 28,29,30,29,28,27,... 3,2,1,0,1,2,3,...28,29,30,29,28,...}

Displays the number of interrupts received by the system (in decimal) on the two right-most
7-segment displays of the development board.
    Use lead-zero blanking on the 7-segment display
*/

.data
sseg: .byte 0x03,0x9F,0x25,0x0D,0x99,0x49,0x41,0x1F,0x01,0x09 # LUT for 7-segs

.text
# s0 = addr for anodes
# s1 = addr for sseg
# s2 = sseg LUT
# a0 = count of intrs
# a1 = BCD intr
# a6 = count down flag
# a7 = interrupt flag
init:
    li s0, 0x1100C008           # addr for anodes
    li s1, 0x1100C004           # addr for sseg
    la s2, sseg                 # addr for sseg LUT
    li s3, 0x1100C000           # addr for switches
    li sp, 0xFFF0               # set stack pointer
    la t0, ISR                  # load ISR address into t0
    csrrw zero, mtvec, t0       # set mtvec to ISR address
    mv a7, zero                 # clear interrupt flag

    li a0, 0                    # clear intr count
    li a1, 0                    # clear BCD intr
unmask:
    li t0, 0x8                  # enable interrupts
    csrrs zero, mstatus, t0     # enable interrupts

backgnd: 
    call disp_7seg              # display 7-seg
    beq a7, zero, backgnd       # wait for interrupt

foregnd:
    call incr_cnt               # continue counting sequence
    li a7, 0                    # clear interrupt flag
    j unmask                    # unmask interrupts

#--------------------------------------------------------------
# subroutine to increment to next in sequence
#    params: a0 = cur count of intrs 
#--------------------------------------------------------------
incr_cnt:
    addi sp, sp, -4             # allocate stack space
    sw ra, 0(sp)                # save return address

    lh t0, 0(s3)                # read switches
    and t0, t0, 1               # get right most switch
    beq t0, zero, count_down    # switch is down
    li t2, 20
    beq t2, t0, incr_cnt_end    # at max count
    addi a0, a0, 1              # inc count
    j incr_cnt_end
count_down:
    beq a0, zero, incr_cnt_end  # clamp at zero
    addi a0, a0, -1

incr_cnt_end:
    lw ra, 0(sp)                # restore return address
    addi sp, sp, 4              # deallocate stack space
    ret                         # return from subroutine

#--------------------------------------------------------------
# subroutine to convert count to BCD
#    params: a0 = count of intrs
#    returns: a1 = BCD of count
#--------------------------------------------------------------
toBCD:
    addi sp, sp, -4             # allocate stack space
    sw ra, 0(sp)                # save return address
    li a1, 0                    # clear BCD
    li t0, 0xA                  # load 10 into t0
    mv t1, a0                   # move count into t1
calc_tens:    
    bltu t1, t0, calc_ones      # if count < 10, go to calc_ones
    addi t1, t1, -10            # subtract 10 from count
    addi a1, a1, 1              # increment tens
    j calc_tens                 # go to calc_tens
calc_ones:
    slli a1, a1, 4              # shift tens to upper nibble
    add a1, a1, t1              # add ones to tens

    lw ra, 0(sp)                # restore return address
    addi sp, sp, 4              # deallocate stack space
    ret                         # return from subroutine

#--------------------------------------------------------------
# subroutine to display 7-seg
#    params: a1 = BCD of count
#--------------------------------------------------------------
disp_7seg:
    addi sp, sp, -4             # allocate stack space
    sw ra, 0(sp)                # save return address
disp_ones:
    li t0, 0xF                  # turn off anodes
    sb t0, 0(s0)                # turn off anodes
    andi t0, a1, 0xF            # mask ones value
    add t0, s2, t0              # add offset to LUT
    lbu t0, 0(t0)               # load ones value
    sb t0, 0(s1)                # display ones value
    li t0, 0xE                  # turn on anode
    sb t0, 0(s0)                # turn on anode
    call delay_ff               # delay
    srli t1, a1, 4              # shift tens to lower nibble
disp_tens:
    beq t1, zero, disp_7seg_end # if tens == 0, return
    li t0, 0xF                  # turn off anodes
    sb t0, 0(s0)                # turn off anodes
    andi t0, t1, 0xF            # mask tens value
    add t0, s2, t0              # add offset to LUT
    lbu t1, 0(t0)               # load tens value
    sb t1, 0(s1)                # display tens value
    li t0, 0xD                  # turn on anode
    sb t0, 0(s0)                # turn on anode
    call delay_ff               # delay
disp_7seg_end:
    lw ra, 0(sp)                # restore return address
    addi sp, sp, 4              # deallocate stack space
    ret                         # return from subroutine

#--------------------------------------------------------------
# Subroutine for delay
# desc: delays for 0xFF cycles for multiplexing
# 
#--------------------------------------------------------------
delay_ff:
    addi sp, sp, -4             # allocate stack space
    sw ra, 0(sp)                # save return address
    li t0, 0xFF                 # delay
    # li t0, 0x1                  # for sim
loop: 
    addi t0, t0, -1             # delay
    bne t0, zero, loop          # delay
    lw ra, 0(sp)                # restore return address
    addi sp, sp, 4              # deallocate stack space
    ret                         # return from subroutine

#--------------------------------------------------------------
# Interrupt Service Routine
#--------------------------------------------------------------
ISR:
    li a7, 1                    # set interrupt flag
    li t2, 0x80                 # clear MPIE
    csrrc zero, mstatus, t2     # clear MPIE
    mret                        # return from interrupt
