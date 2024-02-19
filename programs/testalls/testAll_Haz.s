#This file contains tests for R-type RISCV instructions WITH NO HAZARDS (by this I mean I added nops to prevent hazards)
#You can check to see if any of your tests fail using this link: https://piazza.com/class/kbprd1f9gqj5ro?cid=13

.data
	TESTLOAD: .word 50
	TESTSTORE: .word 0
	TESTU: .word 0x1000770
	
.text
	#Setup stuff
	addi a0, zero, 50 #50 is an arbitrary number, most tests will end up with a result of 50 to keep things simple
	addi a1, zero, 1

R_tests:
	#Add Test
	#initialize t0 to 25, add it to itself, and store that value (50) in a2
	addi t0, zero, 25
	addi a2, zero, 0
	add a2, t0, t0
	bne a0, a2, fail #a2 should equal 50, otherwise program jumps to fail
	

	#Subtract Test
	#Set a2 to 75, then subtract back down to 50
	addi a2, zero, 75
	sub a2, a2, t0
	bne a0, a2, fail
	
	
	#Shift Left Test
	#Shift t0 (25) left once (multiply by 2) and store in a2
	sll a2, t0, a1
	bne a0, a2, fail
	
	
	#Set Less Than Test
	#Check if 25 < 50
	slt a2, t0, a0 #slt stores 1 in a2 if t0 < a0, 0 otherwise
	beq a2, zero, fail
	
	
	#Set Less Than Unsigned Test
	#Check if -75 < 50 (it shouldn't be, since its unsigned)
	addi t0, zero, -75
	sltu a2, t0, a0 #slt stores 1 in a2 if t0 < a0, 0 otherwise
	bne a2, zero, fail
	
	
	#Xor Test
	#Xor something with itself, should be 0
	#Xor something with 0, should be itself
	xor a2, t0, t0
	bne a2, zero, fail
	xor a2, t0, zero
	bne a2, t0, fail
	

	#Shift Right Logical Test
	#Set a2 to 100, shift right once, test to make sure result is 50
	addi a2, zero, 100
	srl a2, a2, a1
	bne a0, a2, fail

	
	#Shift Right Arithmetic Test
	#Shift 100 right, should be 50
	#Shift -100 right, should be -50
	addi t1, zero, -100
	addi t2, zero, -50
	addi a2, zero, 100
	sra a2, a2, a1
	bne a0, a2, fail
	sra a2, t1, a1
	bne a2, t2, fail
	
	
	#Or Test
	#Or something with itself, should be itself
	#Or something with 0, should be itself
	or a2, a0, a0
	bne a0, a2, fail
	or a2, a0, zero
	bne a0, a2, fail
	
	
	#And Test
	#And something with itself, should be itself
	#And something with 0, should be 0
	and a2, a0, a0
	bne a0, a2, fail
	and a2, a0, zero
	bne a2, zero, fail
	

I_tests:
	#Add Immediate Test
	#initialize t0 to 50
	#subtract 50 off of t0
	addi t0, zero, 50
	bne a0, t0, fail #t0 should equal 50, otherwise program jumps to fail
	addi a2, t0, -50
	bne a2, zero, fail
	
	
	#Set Less Than Immediate Test
	#Check if 25 < 50
	addi t0, zero, 25
	slti a2, t0, 50 #slt stores 1 in a2 if t0 < 50, 0 otherwise	
	beq a2, zero, fail
	
	
	#Set Less Than Immediate Unsigned Test
	#Check if -75 < 50 (it shouldn't be, since its unsigned)
	addi t0, zero, -75
	sltiu a2, t0, 50 #slt stores 1 in a2 if t0 < 50, 0 otherwise
	bne a2, zero, fail
	
	
	#Xor Immediate Test
	#Xor something with itself, should be 0
	#Xor something with 0, should be itself
	addi t0, zero, 25
	xori a2, t0, 25
	bne a2, zero, fail
	xori a2, t0, 0
	bne a2, t0, fail
	
	
	#Or Immediate Test
	#Or something with itself, should be itself
	#Or something with 0, should be itself
	ori a2, a0, 50
	bne a0, a2, fail
	ori a2, a0, 0
	bne a0, a2, fail
	
	
	#And Immediate Test
	#And something with itself, should be itself
	#And something with 0, should be 0
	andi a2, a0, 50
	bne a0, a2, fail
	andi a2, a0, 0
	bne a2, zero, fail
	
	
	#Shift Left Immediate Test
	#Shift t0 (25) left once (multiply by 2) and store in a2
	slli a2, t0, 1
	bne a0, a2, fail
	
	
	#Shift Right Logical Immediate Test
	#Set a2 to 100, shift right once, test to make sure result is 50
	addi a2, zero, 100
	srli a2, a2, 1
	bne a0, a2, fail
	
	
	#Shift Right Arithmetic Immediate Test
	#Shift 100 right, should be 50
	#Shift -100 right, should be -50
	addi t1, zero, -100
	addi t2, zero, -50
	addi a2, zero, 100
	srai a2, a2, 1
	bne a0, a2, fail
	srai a2, t1, 1
	bne a2, t2, fail
	
	
B_tests:
	addi t0, zero, -50
	
	
	#BEQ test
	beq zero, a0, fail
	
	
	#BNE test
	bne zero, zero, fail
	
	
	#BLT test
	blt a0, zero, fail
	
	
	#BGE test
	bge zero, a0, fail
	
	
	#BLTU test
	bltu t0, zero, fail
	
	
	#BGEU test
	bgeu zero, t0, fail
	
	
Jump_tests:
	addi t1, zero, 0x154
	
	
	#jal test
	jal t0, skip
	beq zero, zero, fail
	
	
	skip:
	#jalr test
	jalr t0, 0(t1)
	beq zero, zero, fail
	
	
Load_tests:	
	#Load test
	la t0, TESTLOAD
	lw t1, 0(t0)
	bne t1, a0, fail
	
	
STORE_tests:	
	#Store test
	la t0, TESTSTORE
	sw a0, 0(t0)
	lw t1, 0(t0)
	bne t1, a0, fail
	

U_tests:	
	#Load upper immediate test
	la t0, TESTU
	lw t1, 0(t0) #t1 should now hold the value 6000
	lui t2, 4096 #upper 4 bits of 6000
	addi t2, t2, 1904 #this is the lower 12 bits of 6000
	bne t1, t2, fail
	
	
RESET:
	j R_tests
	
fail:
	j fail