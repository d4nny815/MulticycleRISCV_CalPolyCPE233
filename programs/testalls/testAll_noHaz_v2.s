#This file contains tests for R-type RISCV instructions WITH NO HAZARDS (by this I mean I added nops to prevent hazards)

#You can check to see if any of your tests fail using this link: https://piazza.com/class/kbprd1f9gqj5ro?cid=13
# If new instructions are added, there are instructions that need to be adjusted
# For example, in the U_tests section, the first addi points to the 
# memory address of TESTU. For every command added, the immediate values
# need to be adjusted by 4 * number of added commands.

.data
	TESTLOAD: .word 50
	TESTSTORE: .word 0
	TESTU: .word 0x1000770
	
.text
	#Setup stuff
	addi a0, zero, 50 #50 is an arbitrary number, most tests will end up with a result of 50 to keep things simple
	addi a1, zero, 1

R_tests:
	nop
	nop
	#Add Test
	#initialize t0 to 25, add it to itself, and store that value (50) in a2
	addi t0, zero, 25
	addi a2, zero, 0
	nop
	add a2, t0, t0
	nop
	nop
	bne a0, a2, fail #a2 should equal 50, otherwise program jumps to fail
	nop
	nop
	#Subtract Test
	#Set a2 to 75, then subtract back down to 50
	addi a2, zero, 75
	nop
	nop
	sub a2, a2, t0
	nop
	nop
	bne a0, a2, fail
	nop
	nop
	
	#Shift Left Test
	#Shift t0 (25) left once (multiply by 2) and store in a2
	sll a2, t0, a1
	nop
	nop
	bne a0, a2, fail
	nop
	nop
	
	#Set Less Than Test
	#Check if 25 < 50
	slt a2, t0, a0 #slt stores 1 in a2 if t0 < a0, 0 otherwise
	nop
	nop
	beq a2, zero, fail
	nop
	nop
	
	#Set Less Than Unsigned Test
	#Check if -75 < 50 (it shouldn't be, since its unsigned)
	addi t0, zero, -75
	nop
	nop
	sltu a2, t0, a0 #slt stores 1 in a2 if t0 < a0, 0 otherwise
	nop
	nop
	bne a2, zero, fail
	nop
	nop
	
	#Xor Test
	#Xor something with itself, should be 0
	#Xor something with 0, should be itself
	xor a2, t0, t0
	nop
	nop
	bne a2, zero, fail
	nop
	nop
	xor a2, t0, zero
	nop
	nop
	bne a2, t0, fail
	nop
	nop
	
	#Shift Right Logical Test
	#Set a2 to 100, shift right once, test to make sure result is 50
	addi a2, zero, 100
	nop
	nop
	srl a2, a2, a1
	nop
	nop
	bne a0, a2, fail
	nop
	nop
	
	#Shift Right Arithmetic Test
	#Shift 100 right, should be 50
	#Shift -100 right, should be -50
	addi t1, zero, -100
	addi t2, zero, -50
	addi a2, zero, 100
	nop
	nop
	sra a2, a2, a1
	nop
	nop
	bne a0, a2, fail
	nop
	nop
	sra a2, t1, a1
	nop
	nop
	bne a2, t2, fail
	nop
	nop
	
	#Or Test
	#Or something with itself, should be itself
	#Or something with 0, should be itself
	or a2, a0, a0
	nop
	nop
	bne a0, a2, fail
	nop
	nop
	or a2, a0, zero
	nop
	nop
	bne a0, a2, fail
	nop
	nop
	
	#And Test
	#And something with itself, should be itself
	#And something with 0, should be 0
	and a2, a0, a0
	nop
	nop
	bne a0, a2, fail
	nop
	nop
	and a2, a0, zero
	nop
	nop
	bne a2, zero, fail
	nop
	nop

I_tests:
	nop
	nop
	#Add Immediate Test
	#initialize t0 to 50
	#subtract 50 off of t0
	addi t0, zero, 50
	nop
	nop
	bne a0, t0, fail #t0 should equal 50, otherwise program jumps to fail
	nop
	nop
	addi a2, t0, -50
	nop
	nop
	bne a2, zero, fail
	nop
	nop
	
	#Set Less Than Immediate Test
	#Check if 25 < 50
	addi t0, zero, 25
	nop
	nop
	slti a2, t0, 50 #slt stores 1 in a2 if t0 < 50, 0 otherwise
	nop
	nop
	beq a2, zero, fail
	nop
	nop
	
	#Set Less Than Immediate Unsigned Test
	#Check if -75 < 50 (it shouldn't be, since its unsigned)
	addi t0, zero, -75
	nop
	nop
	sltiu a2, t0, 50 #slt stores 1 in a2 if t0 < 50, 0 otherwise
	nop
	nop
	bne a2, zero, fail
	nop
	nop
	
	#Xor Immediate Test
	#Xor something with itself, should be 0
	#Xor something with 0, should be itself
	addi t0, zero, 25
	nop
	nop
	xori a2, t0, 25
	nop
	nop
	bne a2, zero, fail
	nop
	nop
	xori a2, t0, 0
	nop
	nop
	bne a2, t0, fail
	nop
	nop
	
	#Or Immediate Test
	#Or something with itself, should be itself
	#Or something with 0, should be itself
	ori a2, a0, 50
	nop
	nop
	bne a0, a2, fail
	nop
	nop
	ori a2, a0, 0
	nop
	nop
	bne a0, a2, fail
	nop
	nop
	
	#And Immediate Test
	#And something with itself, should be itself
	#And something with 0, should be 0
	andi a2, a0, 50
	nop
	nop
	bne a0, a2, fail
	nop
	nop
	andi a2, a0, 0
	nop
	nop
	bne a2, zero, fail
	nop
	nop
	
	#Shift Left Immediate Test
	#Shift t0 (25) left once (multiply by 2) and store in a2
	slli a2, t0, 1
	nop
	nop
	bne a0, a2, fail
	nop
	nop
	
	#Shift Right Logical Immediate Test
	#Set a2 to 100, shift right once, test to make sure result is 50
	addi a2, zero, 100
	nop
	nop
	srli a2, a2, 1
	nop
	nop
	bne a0, a2, fail
	nop
	nop
	
	#Shift Right Arithmetic Immediate Test
	#Shift 100 right, should be 50
	#Shift -100 right, should be -50
	addi t1, zero, -100
	addi t2, zero, -50
	addi a2, zero, 100
	nop
	nop
	srai a2, a2, 1
	nop
	nop
	bne a0, a2, fail
	nop
	nop
	srai a2, t1, 1
	nop
	nop
	bne a2, t2, fail
	nop
	nop
	
B_tests:
	addi t0, zero, -50
	nop
	nop
	
	#BEQ test
	beq zero, a0, fail
	nop
	nop
	
	#BNE test
	bne zero, zero, fail
	nop
	nop
	
	#BLT test
	blt a0, zero, fail
	nop
	nop
	
	#BGE test
	bge zero, a0, fail
	nop
	nop
	
	#BLTU test
	bltu t0, zero, fail
	nop
	nop
	
	#BGEU test
	bgeu zero, t0, fail
	nop
	nop
	
Jump_tests:
	addi t1, zero, 0x3c8
	nop
	nop
	#jal test
	jal t0, skip
	nop
	nop
	beq zero, zero, fail
	skip:
	nop
	nop
	
	#jalr test
	jalr t0, 0(t1)
	nop
	nop
	beq zero, zero, fail
	nop
	nop
	
Load_tests:	
	nop
	nop
	#Load test
	la t0, TESTLOAD
	nop
	nop
	lw t1, 0(t0)
	nop
	nop
	bne t1, a0, fail
	nop
	nop
	
STORE_tests:	
	nop
	nop
	#Store test
	la t0, TESTSTORE
	nop
	nop
	sw a0, 0(t0)
	nop
	nop
	nop
	nop
	lw t1, 0(t0)
	nop
	nop
	bne t1, a0, fail
	nop
	nop
	
U_tests:	
	nop
	nop
	
	#Load upper immediate test
	la t0, TESTU
	nop
	nop
	lw t1, 0(t0) #t1 should now hold the value 6000
	nop
	nop
	lui t2, 4096 #upper 4 bits of 6000
	nop
	nop
	addi t2, t2, 1904 #this is the lower 12 bits of 6000
	nop
	nop
	bne t1, t2, fail
	nop
	nop
	
RESET:
	j R_tests
	nop
	nop
	
fail:
	j fail
