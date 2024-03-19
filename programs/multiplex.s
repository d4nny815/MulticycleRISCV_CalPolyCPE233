.data
sseg: .byte 0x03,0x9F,0x25,0x0D,0x99,0x49,0x41,0x1F,0x01,0x09   # LUT for 7-segs

.text
init:
    li s0, 0x1100C008           # addr for anodes (active low, MSB is rightmost) 
    li s1, 0x1100C004           # addr for sseg
    la s2, sseg                 # addr for sseg LUT
    li s3, 0x1100C000           # addr for switches
    li sp, 0x10000              # set stack pointer
    # la t0, ISR                  # load ISR address into t0
    # csrrw zero, mtvec, t0       # set mtvec to ISR address
    # mv a7, zero                 # clear interrupt flag

    li a0, 0                    # clear current count
    li a1, 0                    # clear BCD intr

    li a0, 1234                 # load BCD into a1
    li a0, 1009
    call toBCD                  # call toBCD


poll:
    call disp_7seg              # call disp_7seg
    j poll


#--------------------------------------------------------------
# subroutine to convert count to BCD
#    desc: max count is 9999
#    params: a0 = count
#    returns: a1 = BCD of count
#--------------------------------------------------------------
toBCD:
    addi sp, sp, -4             # allocate stack space
    sw ra, 0(sp)                # save return address
    li a1, 0                    # clear BCD
    li t0, 0x1000                # load 1000 into t0
    li t3, 1000
    mv t2, a0                   # move count into t2
calc_thousands:
    bltu t2, t3, calc_hundreds  # if count < 1000, go to calc_hundreds
    addi t2, t2, -1000          # subtract 1000 from count
    add a1, a1, t0              # increment thousands
    j calc_thousands            # go to calc_thousands
calc_hundreds:
    li t0, 100                  # load 100 into t0
calc_hundreds_loop:
    bltu t2, t0, calc_tens      # if count < 100, go to calc_tens
    addi t2, t2, -100           # subtract 100 from count
    addi a1, a1, 0x100          # increment hundreds
    j calc_hundreds_loop        # go to calc_hundreds_loop
calc_tens:
    li t0, 10                   # load 10 into t0
calc_tens_loop:    
    bltu t2, t0, calc_ones      # if count < 10, go to calc_ones
    addi t2, t2, -10            # subtract 10 from count
    addi a1, a1, 0x10           # increment tens
    j calc_tens_loop                 # go to calc_tens
calc_ones:
    add a1, a1, t2              # add ones

    lw ra, 0(sp)                # restore return address
    addi sp, sp, 4              # deallocate stack space
    ret                         # return from subroutine


#--------------------------------------------------------------
# subroutine to display 7-seg
#    params: a1 = BCD of count
#--------------------------------------------------------------
disp_7seg:
    addi sp, sp, -8             # allocate stack space
    sw ra, 0(sp)                # save return address
    sw a1, 4(sp)                # save BCD
    li t6, 0xF
    lbu t5, 0(s2)               # load 0

disp_ones:
    sb t6, 0(s0)                # turn off anodes
    andi t0, a1, 0xF            # mask ones value
    add t0, s2, t0              # add offset to LUT
    lbu t0, 0(t0)               # load ones value
    sb t0, 0(s1)                # display ones value
    li t0, 0x7                  # turn on anode
    sb t0, 0(s0)                # turn on anode
    call delay_ff               # delay

    srli a1, a1, 4              # shift right BCD
    beqz a1, disp_7seg_end      # if nothing left, go to disp_7seg_end
    andi t2, a1, 0xF            # mask tens value

disp_tens:
    sb t6, 0(s0)                # turn off anodes
    bnez t2, disp_not_zero      # if tens != 0, go to disp_not_zero
    sb t5, 0(s1)                # display 0
    j tens_turn_on              # go to tens_turn_on
disp_not_zero:
    add t0, s2, t2              # add offset to LUT
    sb t5, 0(s1)                # display tens value
tens_turn_on:
    li t0, 0xB                  # turn on anode
    sb t0, 0(s0)                # turn on anode
    call delay_ff               # delay

    srli a1, a1, 4              # shift right BCD
    beqz a1, disp_7seg_end      # if nothing left, go to disp_7seg_end
    andi t2, a1, 0xF            # mask hundreds value

disp_hunds:
    sb t6, 0(s0)                # turn off anodes
    bnez t2, disp_not_zero_h    # if hundreds != 0, go to disp_not_zero_h
    sb t5, 0(s1)                # display 0
    j hunds_turn_on             # go to hunds_turn_on
disp_not_zero_h:
    add t0, s2, t2              # add offset to LUT
    lbu t0, 0(t0)               # load hundreds value
    sb t0, 0(s1)                # display hundreds value
hunds_turn_on:
    li t0, 0xD                  # turn on anode
    sb t0, 0(s0)                # turn on anode
    call delay_ff               # delay

    srli a1, a1, 4              # shift right BCD
    beqz a1, disp_7seg_end      # if nothing left, go to disp_7seg_end
    andi t2, a1, 0xF            # mask thousands value
disp_thous:
    sb t6, 0(s0)                # turn off anodes
    add t0, s2, t2              # add offset to LUT
    lbu t0, 0(t0)               # load thousands value
    sb t0, 0(s1)                # display thousands value
    li t0, 0xE                  # turn on anode
    sb t0, 0(s0)                # turn on anode
    call delay_ff               # delay

disp_7seg_end:
    lw a1, 4(sp)                # restore BCD
    lw ra, 0(sp)                # restore return address
    addi sp, sp, 8              # deallocate stack space
    ret                         # return from subroutine


#--------------------------------------------------------------
# Subroutine for delay
# desc: delays for 0xFF cycles for multiplexing
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




