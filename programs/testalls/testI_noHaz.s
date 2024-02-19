#You can check to see if any of your tests fail using this link: https://piazza.com/class/kbprd1f9gqj5ro?cid=13
	
	#Setup stuff
	addi a0, zero, 50 #50 is an arbitrary number, most tests will end up with a result of 50 to keep things simple
	addi a1, zero, 1

tests:
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
	j tests
	
fail:
	j fail
