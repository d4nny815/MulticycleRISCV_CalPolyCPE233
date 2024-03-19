.data
sseg: .byte 0x03,0x9F,0x25,0x0D,0x99,0x49,0x41,0x1F,0x01,0x09   # LUT for 7-segs


.text
init:
    li s0, 0x1100C008           # addr for anodes (active low, MSB is rightmost) 
    li s1, 0x1100C004           # addr for sseg
    la s2, sseg                 # addr for sseg LUT
    li s3, 0x1100C000           # addr for switches
    li s3, 0x1100D000           # TC CSR port address
    li s4, 0x1100D004           # TC count port address
    li sp, 0xfffc               # set stack pointer
    la t0, ISR                  # load ISR address into t0
    csrrw zero, mtvec, t0       # set mtvec to ISR address

    li t0, 0x1000               # blink rate
    sw t0, 0(s4)                # init TC count 
    li t0, 0x01                 # init TC CSR
    sb t0, 0(s3)                # no prescale, turn on TC

    li a6, 0                    # current digit
    li a0, 1002                 # init count
    call toBCD

enable_intr:
    li a7, 0                    # clear interrupt flag
    addi a6, a6, 1              # increment digit
    andi a6, a6, 0x3            # wrap around
    li t0, 0x8                  # enable interrupts
    csrrs zero, mstatus, t0     # enable interrupts

back_gnd:
    beqz a7, back_gnd           # wait for interrupt

fore_gnd:
    call disp_7seg
    j enable_intr

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
    sw a6, 4(sp)                # save digit
    li t6, 0xf
    sb t6, 0(s0)                # turn off all anodes

    # case statement
    beqz a6, disp_ones          # display ones
    addi a6, a6, -1             
    beqz a6, disp_tens          # display tens
    addi a6, a6, -1             
    beqz a6, disp_hunds         # display hundreds
    j disp_thous                # display thousands

disp_ones:
    andi t0, a1, 0xf            # mask out all but ones
    add t0, s2, t0              # get 7-seg value
    lb t0, 0(t0)                # load 7-seg value
    sb t0, 0(s1)                # display 7-seg value
    li t0, 0x7                  # set anode value
    sb t0, 0(s0)                # display anode value
    j disp_ret                  # return

disp_tens:
    srli t0, a1, 4              # shift right to get tens
    beqz t0, disp_ret           # value is less than 10
    andi t0, t0, 0xf            # mask out all but tens
    add t0, s2, t0              # get 7-seg value
    lb t0, 0(t0)                # load 7-seg value
    sb t0, 0(s1)                # display 7-seg value
    li t0, 0xb                  # set anode value
    sb t0, 0(s0)                # display anode value
    j disp_ret                  # return

disp_hunds:
    srli t0, a1, 8              # shift right to get hundreds
    beqz t0, disp_ret           # value is less than 100
    andi t0, t0, 0xf            # mask out all but hundreds
    add t0, s2, t0              # get 7-seg value
    lb t0, 0(t0)                # load 7-seg value
    sb t0, 0(s1)                # display 7-seg value
    li t0, 0xd                  # set anode value
    sb t0, 0(s0)                # display anode value
    j disp_ret                  # return

disp_thous:
    srli t0, a1, 12             # shift right to get thousands
    beqz t0, disp_ret           # value is less than 1000
    andi t0, t0, 0xf            # mask out all but thousands
    add t0, s2, t0              # get 7-seg value
    lb t0, 0(t0)                # load 7-seg value
    sb t0, 0(s1)                # display 7-seg value
    li t0, 0xe                  # set anode value
    sb t0, 0(s0)                # display anode value
    j disp_ret                  # return

disp_ret:
    lw a6, 4(sp)                # restore digit
    lw ra, 0(sp)                # restore return address
    addi sp, sp, 8              # deallocate stack space
    ret                         # return


#--------------------------------------------------------------
# Interrupt Service Routine
#  desc: 
#--------------------------------------------------------------
ISR:
    li a7, 1                    # set interrupt flag
    
    li t2, 0x80                 # clear MPIE
    csrrc zero, mstatus, t2     # clear MPIE
    mret       




