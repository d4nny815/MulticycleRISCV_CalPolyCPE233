#This file contains tests for R-type RISCV instructions WITH NO HAZARDS (by this I mean I added nops to prevent hazards)
#You can check to see if any of your tests fail using this link: https://piazza.com/class/kbprd1f9gqj5ro?cid=13
	
	#Setup stuff
	addi a0, zero, 50 #50 is an arbitrary number, most tests will end up with a result of 50 to keep things simple
	addi a1, zero, 1

tests:
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
	j tests
	
fail:
	j fail
