init:
    # INIT interrupts
    la t0, ISR                  # load ISR address into t0
    csrrw zero, mtvec, t0       # set mtvec to ISR address
    li a7, 0                    # clear intr flag

    li sp, 0xFFFC               # init stack ptr
    li s0, 0x1100C010           # GPU DATA and ADDR. bottom 16b is addr and 12b for color
    li s9, 200                  # display width
    li s10, 150                 # display height

    li a6, 0                    # btn presses
    call blank_screen           # make the screen black
unmask:
    li t0, 0x8                  # enable interrupts
    csrrs zero, mstatus, t0     # enable interrupts

wait_for_btn:
    beq zero, a7, wait_for_btn  # wait for intr
    andi a6, a6, 0x3              # btn_presses mod 4
    call blank_screen

case0:
    bne a6, zero, case1         # btn presses not 0
    li a0, 2                    # x = 2
    li a1, 3                    # y = 3
    li a2, 50                   # length = 50
    li a3, 100                  # height = 100
    call draw_rect_outline
    j endcase
case1:
    li t0, 1
    bne a6, t0, case2           # btn presses not 1
    li a0, 100                  # x = 2
    li a1, 50                   # y = 3
    li a2, 17                   # length = 50
    call draw_Hline
    j endcase
case2:
    li t0, 2
    bne a6, t0, case3           # btn presses not 2
    li a0, 75                   # x = 0
    li a1, 100                  # y = 0
    li a2, 100                  # length = 200
    li a3, 25                   # height = 150
    call draw_rect
    j endcase
case3:                          
    li a0, 100                  # x = 100
    li a1, 0                    # y = 0
    li a2, 100                  # length = 100
    call draw_Vline
endcase:
    addi a6, a6, 1              # increment btn_presses
    li a7, 0                    # clear intr flag
    j unmask



# ===================================
# Description: takes in x_pos and y_pos and outputs the vram addr for GPU
# Registers Used: 
# PARAMS:
#       a6- x_pos
#       a5- y_poss
# Return:
#       a0- 16b addr
# ===================================
calc_vram_addr:
    slli a0, a5, 8              # put y val in pos
    or a0, a0, a6               # combine y and x
    li t0, 0xFFFF               
    and a0, a0, t0              # get rid of anything else
    ret

# ===================================
# Description: takes in addr and pixel data and outputs it GPU
# Registers Used: 
# PARAMS:
#      a0 - pixel data
#      a1 - vram addr
# Return:
#       void
# ===================================
output_pixel:
    slli t0, a0, 16
    or t0, t0, a1
    sw t0, 0(s0)
    ret

# ===================================
# Description: draws a horizontal line at x, y with a length  
# Registers Used: a0, a1, a4, a5, a6
# PARAMS:
#       a0- x_pos
#       a1- y_pos
#       a2- length
# 
# Return:
#       void
# ===================================
draw_Hline:
draw_Hline_save:
    addi sp, sp, -24        # allocate and store on stack
    sw ra, 0(sp)
    sw a0, 4(sp)
    sw a1, 8(sp)
    sw a4, 12(sp)
    sw a5, 16(sp)
    sw a6, 20(sp)

    li a4, 0                # init counter var
    mv a5, a1               # put y_pos in a5
for_loop_Hline:
    lw t1, 4(sp)            # load in starting x_pos
    add t1, t1, a4          # get cur_x
    mv a6, t1               # put cur_x in a6
    call calc_vram_addr
    mv a1, a0               # put vram addr in a1
    li a0, 0xFFF            # white pixel
    call output_pixel
    
for_loop_Hline_admin:
    beq a4, a2, draw_Hline_restore 
    addi a4, a4, 1
    j for_loop_Hline

draw_Hline_restore:
    lw ra, 0(sp)
    lw a0, 4(sp)
    lw a1, 8(sp)
    lw a4, 12(sp)
    lw a5, 16(sp)
    lw a6, 20(sp)
    addi sp, sp, 24         # retrieve from stack
    ret                     # why you lie-ing hehe


# ===================================
# Description: draws a horizontal line at x, y with a height  
# Registers Used: a0, a1, a4, a5, a6
# PARAMS:
#       a0- x_pos
#       a1- y_pos
#       a2- height
# 
# Return:
#       void
# ===================================
draw_Vline:
draw_Vline_save:
    addi sp, sp, -24        # allocate and store on stack
    sw ra, 0(sp)
    sw a0, 4(sp)
    sw a1, 8(sp)
    sw a4, 12(sp)
    sw a5, 16(sp)
    sw a6, 20(sp)

    li a4, 0
    mv a6, a0               # put y_pos in a5
for_loop_Vline:
    lw t0, 8(sp)            # load in starting y_pos
    add t0, t0, a4          # get cur_y from offset
    mv a5, t0               # put cur_y in a5
    call calc_vram_addr
    mv a1, a0               # put vram addr in a1
    li a0, 0xFFF            # pixel data
    call output_pixel 

for_loop_Vline_admin:
    beq a4, a2, draw_Vline_restore 
    addi a4, a4, 1
    j for_loop_Vline

draw_Vline_restore:
    lw ra, 0(sp)
    lw a0, 4(sp)
    lw a1, 8(sp)
    lw a4, 12(sp)
    lw a5, 16(sp)
    lw a6, 20(sp)
    addi sp, sp, 24        # retrieve
    ret                    # why you feeling tall?





# ===================================
# Description: Draw a rectangle outline with 1px width
# Registers Used: 
# PARAMS:
#       a0- start_x
#       a1- start_y
#       a2- length
#       a3- height
# Return:
#       void
# ===================================
draw_rect_outline:
draw_rect_outline_save:
    addi sp, sp, -20
    sw ra, 0(sp)
    sw a0, 4(sp)
    sw a1, 8(sp)
    sw a2, 12(sp)
    sw a3, 16(sp)

h1:
    call draw_Hline
v1:
    add a0, a0, a2              # cur_x = start x + length
    mv a2, a3                   # setup a2 with height
    call draw_Vline
h2:
    add a1, a1, a2              # cur_y = start_y + height
    lw a2, 12(sp)               # load in length
    sub a0, a0, a2              # cur_x = cur_x - length
    call draw_Hline                
v2:
    mv a2, a3                   # setup a2 with height
    sub a1, a1, a2              # cur_y = cur_y - height
    call draw_Vline

draw_rect_outline_restore:
    lw ra, 0(sp)
    lw a0, 4(sp)
    lw a1, 8(sp)
    lw a2, 12(sp)
    lw a3, 16(sp)
    addi sp, sp, -20
    ret

# ===================================
# Description: draw a rectangle filled in at x, y with a length and height
# Registers Used: a1 a4
# PARAMS:
#       a0- start_x     
#       a1- start_y
#       a2- length
#       a3- height
# Return:
#       void
# ===================================
draw_rect:
draw_rect_save:
    addi sp, sp, -12
    sw ra, 0(sp)
    sw a4, 4(sp)
    sw a1, 8(sp)

    li a4, 0                # init offset counter
for_loop_rect:
    lw a1, 8(sp)            # load start_y
    add a1, a1, a4          # cur_y = start_y + offset
    call draw_Hline

for_loop_rect_admin:
    beq a4, a3, draw_rect_restore 
    addi a4, a4, 1
    j for_loop_rect

draw_rect_restore:
    lw ra, 0(sp)
    lw a4, 4(sp)
    lw a1, 8(sp)
    addi sp, sp, 12
    ret

blank_screen:
blank_screen_save:
    addi sp, sp, -20
    sw ra, 0(sp)
    sw a6, 4(sp)
    sw a5, 8(sp) 
    sw a1, 12(sp)
    sw a0, 16(sp)

    li a5, 0                        # v_pixel
v_loop:
    li a6, 0                        # h_pixel
h_loop:
    call calc_vram_addr             # calculate pixel addr
    mv a1, a0                       # move addr to a1
    li a0, 0                        # balack pixel
    # li a0, 0xfff                    # white pixel
    call output_pixel               # store pixel data in GPU VRAM

h_loop_admin:
    addi a6, a6, 1                  # increment h_addr
    bne a6, s9, h_loop              # continue h_loop unless at max h_addr
v_loop_admin:
    addi a5, a5, 1                  # increment v_addr
    bne a5, s10, v_loop
blank_screen_restore:
    lw ra, 0(sp)
    lw a6, 4(sp)
    lw a5, 8(sp) 
    lw a1, 12(sp)
    lw a0, 16(sp)
    addi sp, sp, 20
    ret

#--------------------------------------------------------------
# Interrupt Service Routine
#  desc: set intr flag
#--------------------------------------------------------------
ISR:
    li a7, 1                    # set interrupt flag
    li t2, 0x80                 # clear MPIE
    csrrc zero, mstatus, t2     # clear MPIE
    mret                        # return from interrupt
