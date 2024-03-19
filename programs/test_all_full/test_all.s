#########################################
#										#
#	OTTER Tests							#
#	(37 instructions)					#
#	CalPoly 233 - Callenes				#
#										#	
#########################################

.equ test, 0x0044 
.equ SWITCHES, 0x11000000
.equ VGA_READ, 0x11040000
.equ LEDS,0x11080000
.equ SSEG, 0x110C0000
#.equ VGA_ADDR, 0x11100000
#.equ VGA_COLOR, 0x11140000


.text

.global main 
.type main, @function
main:
	li x16, SWITCHES
	li x11, SSEG
	li x12, LEDS
loop: 
	li x14,-1; call post_test		#Test 	Number of Tests
	jal x31, add_test; call post_test   	#0	17
	jal x31, sub_test; call post_test	#1	16
	jal x31, and_test; call post_test	#2	7
	jal x31, or_test; call post_test	#3	7
	jal x31, xor_test; call post_test	#4	7
	jal x31, sll_test; call post_test	#5	19
	jal x31, srl_test; call post_test	#6	17
	jal x31, sra_test; call post_test	#7	20
	jal x31, slt_test; call post_test	#8	16
	jal x31, sltu_test; call post_test	#9	16
	jal x31, addi_test; call post_test	#A	16
	jal x31, andi_test; call post_test	#B	5
	jal x31, ori_test; call post_test	#C	5
	jal x31, xori_test; call post_test	#D	5
	jal x31, slli_test; call post_test	#E	15
	jal x31, srli_test; call post_test	#F	16
	jal x31, srai_test; call post_test	#10	16
	jal x31, sltiu_test; call post_test	#11	16
	jal x31, slti_test; call post_test	#12	16
	jal x31, lw_test; call post_test	#13	9
	jal x31, sw_test; call post_test	#14	9
	jal x31, beq_test; call post_test	#15	7
	jal x31, bne_test; call post_test	#16	7
	jal x31, blt_test; call post_test	#17	7
	jal x31, bge_test; call post_test	#18	10
	jal x31, bltu_test; call post_test	#19	7
	jal x31, bgeu_test; call post_test	#1A	10
	jal x31, auipc_test; call post_test	#1B	2
	jal x31, lui_test; call post_test	#1C	5
	jal x31, jal_test; call post_test	#1D	2
	jal x31, jalr_test; call post_test	#1E	4
	jal x31, lh_test; call post_test	#1F	10
	jal x31, lb_test; call post_test	#20	10
	jal x31, lhu_test; call post_test	#21	10
	jal x31, lbu_test; call post_test	#22	10
	jal x31, sh_test; call post_test	#23	10
	jal x31, sb_test; call post_test	#24	10

	j loop
end:

post_test:
	1: lw x13,0(x16) #wait till switch one is off
	andi x13,x13,1
	bnez x13, 1b
	addi x14,x14,1 #change sseg for next test
	sw x14,0(x11)
	ret

update_test:
	slli x10,x10,1; #turn on one more LED
	addi x10,x10,1; 
	sw x10, 0(x12);
	ret

fail: 
 	li x15,-1 # Display error(-1) on SSEG
 	sw x15,0(x11)	
 	jr x31 		# return from current test


add_test:  li x10,0; sw x10, 0(x12)	#17 Tests	
  #-------------------------------------------------------------
  # Arithmetic tests 
  #-------------------------------------------------------------
#slli x10,x10,1;

  li x1, ((0x00000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x00000000) & ((1 << (32 - 1) << 1) - 1)); add x30, x1, x2;; li x29, ((0x00000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 2; bne x30, x29, fail; call update_test;
  li x1, ((0x00000001) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x00000001) & ((1 << (32 - 1) << 1) - 1)); add x30, x1, x2;; li x29, ((0x00000002) & ((1 << (32 - 1) << 1) - 1)); li gp, 3; bne x30, x29, fail; call update_test;
  li x1, ((0x00000003) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x00000007) & ((1 << (32 - 1) << 1) - 1)); add x30, x1, x2;; li x29, ((0x0000000a) & ((1 << (32 - 1) << 1) - 1)); li gp, 4; bne x30, x29, fail; call update_test;

  li x1, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xffffffffffff8000) & ((1 << (32 - 1) << 1) - 1)); add x30, x1, x2;; li x29, ((0xffffffffffff8000) & ((1 << (32 - 1) << 1) - 1)); li gp, 5; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x00000000) & ((1 << (32 - 1) << 1) - 1)); add x30, x1, x2;; li x29, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 6; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xffffffffffff8000) & ((1 << (32 - 1) << 1) - 1)); add x30, x1, x2;; li x29, ((0xffffffff7fff8000) & ((1 << (32 - 1) << 1) - 1)); li gp, 7; bne x30, x29, fail; call update_test;

  li x1, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0000000000007fff) & ((1 << (32 - 1) << 1) - 1)); add x30, x1, x2;; li x29, ((0x0000000000007fff) & ((1 << (32 - 1) << 1) - 1)); li gp, 8; bne x30, x29, fail; call update_test;
  li x1, ((0x000000007fffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); add x30, x1, x2;; li x29, ((0x000000007fffffff) & ((1 << (32 - 1) << 1) - 1)); li gp, 9; bne x30, x29, fail; call update_test;
  li x1, ((0x000000007fffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0000000000007fff) & ((1 << (32 - 1) << 1) - 1)); add x30, x1, x2;; li x29, ((0x0000000080007ffe) & ((1 << (32 - 1) << 1) - 1)); li gp, 10; bne x30, x29, fail; call update_test;

  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0000000000007fff) & ((1 << (32 - 1) << 1) - 1)); add x30, x1, x2;; li x29, ((0xffffffff80007fff) & ((1 << (32 - 1) << 1) - 1)); li gp, 11; bne x30, x29, fail; call update_test;
  li x1, ((0x000000007fffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xffffffffffff8000) & ((1 << (32 - 1) << 1) - 1)); add x30, x1, x2;; li x29, ((0x000000007fff7fff) & ((1 << (32 - 1) << 1) - 1)); li gp, 12; bne x30, x29, fail; call update_test;

  li x1, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); add x30, x1, x2;; li x29, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); li gp, 13; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0000000000000001) & ((1 << (32 - 1) << 1) - 1)); add x30, x1, x2;; li x29, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 14; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); add x30, x1, x2;; li x29, ((0xfffffffffffffffe) & ((1 << (32 - 1) << 1) - 1)); li gp, 15; bne x30, x29, fail; call update_test;

  li x1, ((0x0000000000000001) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x000000007fffffff) & ((1 << (32 - 1) << 1) - 1)); add x30, x1, x2;; li x29, ((0x0000000080000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 16; bne x30, x29, fail; call update_test;
 
  #-------------------------------------------------------------
  # Source/Destination tests
  #-------------------------------------------------------------

  li x1, ((13) & ((1 << (32 - 1) << 1) - 1)); li x2, ((11) & ((1 << (32 - 1) << 1) - 1)); add x1, x1, x2;; li x29, ((24) & ((1 << (32 - 1) << 1) - 1)); li gp, 17; bne x1, x29, fail; call update_test;
  li x1, ((14) & ((1 << (32 - 1) << 1) - 1)); li x2, ((11) & ((1 << (32 - 1) << 1) - 1)); add x2, x1, x2;; li x29, ((25) & ((1 << (32 - 1) << 1) - 1)); li gp, 18; bne x2, x29, fail; call update_test;

	jr x31


sub_test:  li x10,0; sw x10, 0(x12)	#16 Tests

  #-------------------------------------------------------------
  # Arithmetic tests
  #-------------------------------------------------------------

  li x1, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); sub x30, x1, x2;; li x29, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 2; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000000000001) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0000000000000001) & ((1 << (32 - 1) << 1) - 1)); sub x30, x1, x2;; li x29, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 3; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000000000003) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0000000000000007) & ((1 << (32 - 1) << 1) - 1)); sub x30, x1, x2;; li x29, ((0xfffffffffffffffc) & ((1 << (32 - 1) << 1) - 1)); li gp, 4; bne x30, x29, fail; call update_test;

  li x1, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xffffffffffff8000) & ((1 << (32 - 1) << 1) - 1)); sub x30, x1, x2;; li x29, ((0x0000000000008000) & ((1 << (32 - 1) << 1) - 1)); li gp, 5; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); sub x30, x1, x2;; li x29, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 6; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xffffffffffff8000) & ((1 << (32 - 1) << 1) - 1)); sub x30, x1, x2;; li x29, ((0xffffffff80008000) & ((1 << (32 - 1) << 1) - 1)); li gp, 7; bne x30, x29, fail; call update_test;

  li x1, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0000000000007fff) & ((1 << (32 - 1) << 1) - 1)); sub x30, x1, x2;; li x29, ((0xffffffffffff8001) & ((1 << (32 - 1) << 1) - 1)); li gp, 8; bne x30, x29, fail; call update_test;
  li x1, ((0x000000007fffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); sub x30, x1, x2;; li x29, ((0x000000007fffffff) & ((1 << (32 - 1) << 1) - 1)); li gp, 9; bne x30, x29, fail; call update_test;
  li x1, ((0x000000007fffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0000000000007fff) & ((1 << (32 - 1) << 1) - 1)); sub x30, x1, x2;; li x29, ((0x000000007fff8000) & ((1 << (32 - 1) << 1) - 1)); li gp, 10; bne x30, x29, fail; call update_test;

  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0000000000007fff) & ((1 << (32 - 1) << 1) - 1)); sub x30, x1, x2;; li x29, ((0xffffffff7fff8001) & ((1 << (32 - 1) << 1) - 1)); li gp, 11; bne x30, x29, fail; call update_test;
  li x1, ((0x000000007fffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xffffffffffff8000) & ((1 << (32 - 1) << 1) - 1)); sub x30, x1, x2;; li x29, ((0x0000000080007fff) & ((1 << (32 - 1) << 1) - 1)); li gp, 12; bne x30, x29, fail; call update_test;

  li x1, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); sub x30, x1, x2;; li x29, ((0x0000000000000001) & ((1 << (32 - 1) << 1) - 1)); li gp, 13; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0000000000000001) & ((1 << (32 - 1) << 1) - 1)); sub x30, x1, x2;; li x29, ((0xfffffffffffffffe) & ((1 << (32 - 1) << 1) - 1)); li gp, 14; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); sub x30, x1, x2;; li x29, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 15; bne x30, x29, fail; call update_test;

  #-------------------------------------------------------------
  # Source/Destination tests
  #-------------------------------------------------------------

  li x1, ((13) & ((1 << (32 - 1) << 1) - 1)); li x2, ((11) & ((1 << (32 - 1) << 1) - 1)); sub x1, x1, x2;; li x29, ((2) & ((1 << (32 - 1) << 1) - 1)); li gp, 16; bne x1, x29, fail; call update_test;
  li x1, ((14) & ((1 << (32 - 1) << 1) - 1)); li x2, ((11) & ((1 << (32 - 1) << 1) - 1)); sub x2, x1, x2;; li x29, ((3) & ((1 << (32 - 1) << 1) - 1)); li gp, 17; bne x2, x29, fail; call update_test;

jr x31

and_test:  li x10,0; sw x10, 0(x12)	# 7 tests


  #-------------------------------------------------------------
  # Logical tests
  #-------------------------------------------------------------

  li x1, ((0xff00ff00) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0f0f0f0f) & ((1 << (32 - 1) << 1) - 1)); and x30, x1, x2;; li x29, ((0x0f000f00) & ((1 << (32 - 1) << 1) - 1)); li gp, 2; bne x30, x29, fail; call update_test;
  li x1, ((0x0ff00ff0) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xf0f0f0f0) & ((1 << (32 - 1) << 1) - 1)); and x30, x1, x2;; li x29, ((0x00f000f0) & ((1 << (32 - 1) << 1) - 1)); li gp, 3; bne x30, x29, fail; call update_test;
  li x1, ((0x00ff00ff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0f0f0f0f) & ((1 << (32 - 1) << 1) - 1)); and x30, x1, x2;; li x29, ((0x000f000f) & ((1 << (32 - 1) << 1) - 1)); li gp, 4; bne x30, x29, fail; call update_test;
  li x1, ((0xf00ff00f) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xf0f0f0f0) & ((1 << (32 - 1) << 1) - 1)); and x30, x1, x2;; li x29, ((0xf000f000) & ((1 << (32 - 1) << 1) - 1)); li gp, 5; bne x30, x29, fail; call update_test;

  #-------------------------------------------------------------
  # Source/Destination tests
  #-------------------------------------------------------------

  li x1, ((0xff00ff00) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0f0f0f0f) & ((1 << (32 - 1) << 1) - 1)); and x1, x1, x2;; li x29, ((0x0f000f00) & ((1 << (32 - 1) << 1) - 1)); li gp, 6; bne x1, x29, fail; call update_test;
  li x1, ((0x0ff00ff0) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xf0f0f0f0) & ((1 << (32 - 1) << 1) - 1)); and x2, x1, x2;; li x29, ((0x00f000f0) & ((1 << (32 - 1) << 1) - 1)); li gp, 7; bne x2, x29, fail; call update_test;
  li x1, ((0xff00ff00) & ((1 << (32 - 1) << 1) - 1)); and x1, x1, x1;; li x29, ((0xff00ff00) & ((1 << (32 - 1) << 1) - 1)); li gp, 8; bne x1, x29, fail; call update_test;

jr x31

or_test:  li x10,0; sw x10, 0(x12)	#7 tests

  #-------------------------------------------------------------
  # Logical tests
  #-------------------------------------------------------------

  li x1, ((0xff00ff00) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0f0f0f0f) & ((1 << (32 - 1) << 1) - 1)); or x30, x1, x2;; li x29, ((0xff0fff0f) & ((1 << (32 - 1) << 1) - 1)); li gp, 2; bne x30, x29, fail; call update_test;
  li x1, ((0x0ff00ff0) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xf0f0f0f0) & ((1 << (32 - 1) << 1) - 1)); or x30, x1, x2;; li x29, ((0xfff0fff0) & ((1 << (32 - 1) << 1) - 1)); li gp, 3; bne x30, x29, fail; call update_test;
  li x1, ((0x00ff00ff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0f0f0f0f) & ((1 << (32 - 1) << 1) - 1)); or x30, x1, x2;; li x29, ((0x0fff0fff) & ((1 << (32 - 1) << 1) - 1)); li gp, 4; bne x30, x29, fail; call update_test;
  li x1, ((0xf00ff00f) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xf0f0f0f0) & ((1 << (32 - 1) << 1) - 1)); or x30, x1, x2;; li x29, ((0xf0fff0ff) & ((1 << (32 - 1) << 1) - 1)); li gp, 5; bne x30, x29, fail; call update_test;

  #-------------------------------------------------------------
  # Source/Destination tests
  #-------------------------------------------------------------

  li x1, ((0xff00ff00) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0f0f0f0f) & ((1 << (32 - 1) << 1) - 1)); or x1, x1, x2;; li x29, ((0xff0fff0f) & ((1 << (32 - 1) << 1) - 1)); li gp, 6; bne x1, x29, fail; call update_test;
  li x1, ((0xff00ff00) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0f0f0f0f) & ((1 << (32 - 1) << 1) - 1)); or x2, x1, x2;; li x29, ((0xff0fff0f) & ((1 << (32 - 1) << 1) - 1)); li gp, 7; bne x2, x29, fail; call update_test;
  li x1, ((0xff00ff00) & ((1 << (32 - 1) << 1) - 1)); or x1, x1, x1;; li x29, ((0xff00ff00) & ((1 << (32 - 1) << 1) - 1)); li gp, 8; bne x1, x29, fail; call update_test;

jr x31

xor_test:  li x10,0; sw x10, 0(x12)	#7 test
	
  #-------------------------------------------------------------
  # Logical tests
  #-------------------------------------------------------------

  li x1, ((0xff00ff00) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0f0f0f0f) & ((1 << (32 - 1) << 1) - 1)); xor x30, x1, x2;; li x29, ((0xf00ff00f) & ((1 << (32 - 1) << 1) - 1)); li gp, 2; bne x30, x29, fail; call update_test;
  li x1, ((0x0ff00ff0) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xf0f0f0f0) & ((1 << (32 - 1) << 1) - 1)); xor x30, x1, x2;; li x29, ((0xff00ff00) & ((1 << (32 - 1) << 1) - 1)); li gp, 3; bne x30, x29, fail; call update_test;
  li x1, ((0x00ff00ff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0f0f0f0f) & ((1 << (32 - 1) << 1) - 1)); xor x30, x1, x2;; li x29, ((0x0ff00ff0) & ((1 << (32 - 1) << 1) - 1)); li gp, 4; bne x30, x29, fail; call update_test;
  li x1, ((0xf00ff00f) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xf0f0f0f0) & ((1 << (32 - 1) << 1) - 1)); xor x30, x1, x2;; li x29, ((0x00ff00ff) & ((1 << (32 - 1) << 1) - 1)); li gp, 5; bne x30, x29, fail; call update_test;

  #-------------------------------------------------------------
  # Source/Destination tests
  #-------------------------------------------------------------

  li x1, ((0xff00ff00) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0f0f0f0f) & ((1 << (32 - 1) << 1) - 1)); xor x1, x1, x2;; li x29, ((0xf00ff00f) & ((1 << (32 - 1) << 1) - 1)); li gp, 6; bne x1, x29, fail; call update_test;
  li x1, ((0xff00ff00) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0f0f0f0f) & ((1 << (32 - 1) << 1) - 1)); xor x2, x1, x2;; li x29, ((0xf00ff00f) & ((1 << (32 - 1) << 1) - 1)); li gp, 7; bne x2, x29, fail; call update_test;
  li x1, ((0xff00ff00) & ((1 << (32 - 1) << 1) - 1)); xor x1, x1, x1;; li x29, ((0x00000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 8; bne x1, x29, fail; call update_test;

jr x31

sll_test:  li x10,0; sw x10, 0(x12)	#19 tests

  #-------------------------------------------------------------
  # Arithmetic tests
  #-------------------------------------------------------------

  li x1, ((0x0000000000000001) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0) & ((1 << (32 - 1) << 1) - 1)); sll x30, x1, x2;; li x29, ((0x0000000000000001) & ((1 << (32 - 1) << 1) - 1)); li gp, 2; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000000000001) & ((1 << (32 - 1) << 1) - 1)); li x2, ((1) & ((1 << (32 - 1) << 1) - 1)); sll x30, x1, x2;; li x29, ((0x0000000000000002) & ((1 << (32 - 1) << 1) - 1)); li gp, 3; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000000000001) & ((1 << (32 - 1) << 1) - 1)); li x2, ((7) & ((1 << (32 - 1) << 1) - 1)); sll x30, x1, x2;; li x29, ((0x0000000000000080) & ((1 << (32 - 1) << 1) - 1)); li gp, 4; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000000000001) & ((1 << (32 - 1) << 1) - 1)); li x2, ((14) & ((1 << (32 - 1) << 1) - 1)); sll x30, x1, x2;; li x29, ((0x0000000000004000) & ((1 << (32 - 1) << 1) - 1)); li gp, 5; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000000000001) & ((1 << (32 - 1) << 1) - 1)); li x2, ((31) & ((1 << (32 - 1) << 1) - 1)); sll x30, x1, x2;; li x29, ((0x0000000080000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 6; bne x30, x29, fail; call update_test;

  li x1, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0) & ((1 << (32 - 1) << 1) - 1)); sll x30, x1, x2;; li x29, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); li gp, 7; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((1) & ((1 << (32 - 1) << 1) - 1)); sll x30, x1, x2;; li x29, ((0xfffffffffffffffe) & ((1 << (32 - 1) << 1) - 1)); li gp, 8; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((7) & ((1 << (32 - 1) << 1) - 1)); sll x30, x1, x2;; li x29, ((0xffffffffffffff80) & ((1 << (32 - 1) << 1) - 1)); li gp, 9; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((14) & ((1 << (32 - 1) << 1) - 1)); sll x30, x1, x2;; li x29, ((0xffffffffffffc000) & ((1 << (32 - 1) << 1) - 1)); li gp, 10; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((31) & ((1 << (32 - 1) << 1) - 1)); sll x30, x1, x2;; li x29, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 11; bne x30, x29, fail; call update_test;

  li x1, ((0x0000000021212121) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0) & ((1 << (32 - 1) << 1) - 1)); sll x30, x1, x2;; li x29, ((0x0000000021212121) & ((1 << (32 - 1) << 1) - 1)); li gp, 12; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000021212121) & ((1 << (32 - 1) << 1) - 1)); li x2, ((1) & ((1 << (32 - 1) << 1) - 1)); sll x30, x1, x2;; li x29, ((0x0000000042424242) & ((1 << (32 - 1) << 1) - 1)); li gp, 13; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000021212121) & ((1 << (32 - 1) << 1) - 1)); li x2, ((7) & ((1 << (32 - 1) << 1) - 1)); sll x30, x1, x2;; li x29, ((0x0000001090909080) & ((1 << (32 - 1) << 1) - 1)); li gp, 14; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000021212121) & ((1 << (32 - 1) << 1) - 1)); li x2, ((14) & ((1 << (32 - 1) << 1) - 1)); sll x30, x1, x2;; li x29, ((0x0000084848484000) & ((1 << (32 - 1) << 1) - 1)); li gp, 15; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000021212121) & ((1 << (32 - 1) << 1) - 1)); li x2, ((31) & ((1 << (32 - 1) << 1) - 1)); sll x30, x1, x2;; li x29, ((0x1090909080000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 16; bne x30, x29, fail; call update_test;

  # Verify that shifts only use bottom six bits

  li x1, ((0x0000000021212121) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xffffffffffffffc0) & ((1 << (32 - 1) << 1) - 1)); sll x30, x1, x2;; li x29, ((0x0000000021212121) & ((1 << (32 - 1) << 1) - 1)); li gp, 17; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000021212121) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xffffffffffffffc1) & ((1 << (32 - 1) << 1) - 1)); sll x30, x1, x2;; li x29, ((0x0000000042424242) & ((1 << (32 - 1) << 1) - 1)); li gp, 18; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000021212121) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xffffffffffffffc7) & ((1 << (32 - 1) << 1) - 1)); sll x30, x1, x2;; li x29, ((0x0000001090909080) & ((1 << (32 - 1) << 1) - 1)); li gp, 19; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000021212121) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xffffffffffffffce) & ((1 << (32 - 1) << 1) - 1)); sll x30, x1, x2;; li x29, ((0x0000084848484000) & ((1 << (32 - 1) << 1) - 1)); li gp, 20; bne x30, x29, fail; call update_test;

jr x31

srl_test:  li x10,0; sw x10, 0(x12)	#17 tests

  #-------------------------------------------------------------
  # Arithmetic tests
  #-------------------------------------------------------------

  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0) & ((1 << (32 - 1) << 1) - 1)); srl x30, x1, x2;; li x29, ((((0xffffffff80000000) & ((1 << (32 -1) << 1) - 1)) >> (0)) & ((1 << (32 - 1) << 1) - 1)); li gp, 2; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((1) & ((1 << (32 - 1) << 1) - 1)); srl x30, x1, x2;; li x29, ((((0xffffffff80000000) & ((1 << (32 -1) << 1) - 1)) >> (1)) & ((1 << (32 - 1) << 1) - 1)); li gp, 3; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((7) & ((1 << (32 - 1) << 1) - 1)); srl x30, x1, x2;; li x29, ((((0xffffffff80000000) & ((1 << (32 -1) << 1) - 1)) >> (7)) & ((1 << (32 - 1) << 1) - 1)); li gp, 4; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((14) & ((1 << (32 - 1) << 1) - 1)); srl x30, x1, x2;; li x29, ((((0xffffffff80000000) & ((1 << (32 -1) << 1) - 1)) >> (14)) & ((1 << (32 - 1) << 1) - 1)); li gp, 5; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff80000001) & ((1 << (32 - 1) << 1) - 1)); li x2, ((31) & ((1 << (32 - 1) << 1) - 1)); srl x30, x1, x2;; li x29, ((((0xffffffff80000001) & ((1 << (32 -1) << 1) - 1)) >> (31)) & ((1 << (32 - 1) << 1) - 1)); li gp, 6; bne x30, x29, fail; call update_test;

  li x1, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0) & ((1 << (32 - 1) << 1) - 1)); srl x30, x1, x2;; li x29, ((((0xffffffffffffffff) & ((1 << (32 -1) << 1) - 1)) >> (0)) & ((1 << (32 - 1) << 1) - 1)); li gp, 7; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((1) & ((1 << (32 - 1) << 1) - 1)); srl x30, x1, x2;; li x29, ((((0xffffffffffffffff) & ((1 << (32 -1) << 1) - 1)) >> (1)) & ((1 << (32 - 1) << 1) - 1)); li gp, 8; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((7) & ((1 << (32 - 1) << 1) - 1)); srl x30, x1, x2;; li x29, ((((0xffffffffffffffff) & ((1 << (32 -1) << 1) - 1)) >> (7)) & ((1 << (32 - 1) << 1) - 1)); li gp, 9; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((14) & ((1 << (32 - 1) << 1) - 1)); srl x30, x1, x2;; li x29, ((((0xffffffffffffffff) & ((1 << (32 -1) << 1) - 1)) >> (14)) & ((1 << (32 - 1) << 1) - 1)); li gp, 10; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((31) & ((1 << (32 - 1) << 1) - 1)); srl x30, x1, x2;; li x29, ((((0xffffffffffffffff) & ((1 << (32 -1) << 1) - 1)) >> (31)) & ((1 << (32 - 1) << 1) - 1)); li gp, 11; bne x30, x29, fail; call update_test;

  li x1, ((0x0000000021212121) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0) & ((1 << (32 - 1) << 1) - 1)); srl x30, x1, x2;; li x29, ((((0x0000000021212121) & ((1 << (32 -1) << 1) - 1)) >> (0)) & ((1 << (32 - 1) << 1) - 1)); li gp, 12; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000021212121) & ((1 << (32 - 1) << 1) - 1)); li x2, ((1) & ((1 << (32 - 1) << 1) - 1)); srl x30, x1, x2;; li x29, ((((0x0000000021212121) & ((1 << (32 -1) << 1) - 1)) >> (1)) & ((1 << (32 - 1) << 1) - 1)); li gp, 13; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000021212121) & ((1 << (32 - 1) << 1) - 1)); li x2, ((7) & ((1 << (32 - 1) << 1) - 1)); srl x30, x1, x2;; li x29, ((((0x0000000021212121) & ((1 << (32 -1) << 1) - 1)) >> (7)) & ((1 << (32 - 1) << 1) - 1)); li gp, 14; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000021212121) & ((1 << (32 - 1) << 1) - 1)); li x2, ((14) & ((1 << (32 - 1) << 1) - 1)); srl x30, x1, x2;; li x29, ((((0x0000000021212121) & ((1 << (32 -1) << 1) - 1)) >> (14)) & ((1 << (32 - 1) << 1) - 1)); li gp, 15; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000021212121) & ((1 << (32 - 1) << 1) - 1)); li x2, ((31) & ((1 << (32 - 1) << 1) - 1)); srl x30, x1, x2;; li x29, ((((0x0000000021212121) & ((1 << (32 -1) << 1) - 1)) >> (31)) & ((1 << (32 - 1) << 1) - 1)); li gp, 16; bne x30, x29, fail; call update_test;

  # Verify that shifts only use bottom five bits

  li x1, ((0x0000000021212121) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xffffffffffffffc0) & ((1 << (32 - 1) << 1) - 1)); srl x30, x1, x2;; li x29, ((0x0000000021212121) & ((1 << (32 - 1) << 1) - 1)); li gp, 17; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000021212121) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xffffffffffffffc1) & ((1 << (32 - 1) << 1) - 1)); srl x30, x1, x2;; li x29, ((0x0000000010909090) & ((1 << (32 - 1) << 1) - 1)); li gp, 18; bne x30, x29, fail; call update_test;

jr x31

sra_test:  li x10,0; sw x10, 0(x12)		#20 tests

  #-------------------------------------------------------------
  # Arithmetic tests
  #-------------------------------------------------------------

  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0) & ((1 << (32 - 1) << 1) - 1)); sra x30, x1, x2;; li x29, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 2; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((1) & ((1 << (32 - 1) << 1) - 1)); sra x30, x1, x2;; li x29, ((0xffffffffc0000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 3; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((7) & ((1 << (32 - 1) << 1) - 1)); sra x30, x1, x2;; li x29, ((0xffffffffff000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 4; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((14) & ((1 << (32 - 1) << 1) - 1)); sra x30, x1, x2;; li x29, ((0xfffffffffffe0000) & ((1 << (32 - 1) << 1) - 1)); li gp, 5; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff80000001) & ((1 << (32 - 1) << 1) - 1)); li x2, ((31) & ((1 << (32 - 1) << 1) - 1)); sra x30, x1, x2;; li x29, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); li gp, 6; bne x30, x29, fail; call update_test;

  li x1, ((0x000000007fffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0) & ((1 << (32 - 1) << 1) - 1)); sra x30, x1, x2;; li x29, ((0x000000007fffffff) & ((1 << (32 - 1) << 1) - 1)); li gp, 7; bne x30, x29, fail; call update_test;
  li x1, ((0x000000007fffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((1) & ((1 << (32 - 1) << 1) - 1)); sra x30, x1, x2;; li x29, ((0x000000003fffffff) & ((1 << (32 - 1) << 1) - 1)); li gp, 8; bne x30, x29, fail; call update_test;
  li x1, ((0x000000007fffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((7) & ((1 << (32 - 1) << 1) - 1)); sra x30, x1, x2;; li x29, ((0x0000000000ffffff) & ((1 << (32 - 1) << 1) - 1)); li gp, 9; bne x30, x29, fail; call update_test;
  li x1, ((0x000000007fffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((14) & ((1 << (32 - 1) << 1) - 1)); sra x30, x1, x2;; li x29, ((0x000000000001ffff) & ((1 << (32 - 1) << 1) - 1)); li gp, 10; bne x30, x29, fail; call update_test;
  li x1, ((0x000000007fffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((31) & ((1 << (32 - 1) << 1) - 1)); sra x30, x1, x2;; li x29, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 11; bne x30, x29, fail; call update_test;

  li x1, ((0xffffffff81818181) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0) & ((1 << (32 - 1) << 1) - 1)); sra x30, x1, x2;; li x29, ((0xffffffff81818181) & ((1 << (32 - 1) << 1) - 1)); li gp, 12; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff81818181) & ((1 << (32 - 1) << 1) - 1)); li x2, ((1) & ((1 << (32 - 1) << 1) - 1)); sra x30, x1, x2;; li x29, ((0xffffffffc0c0c0c0) & ((1 << (32 - 1) << 1) - 1)); li gp, 13; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff81818181) & ((1 << (32 - 1) << 1) - 1)); li x2, ((7) & ((1 << (32 - 1) << 1) - 1)); sra x30, x1, x2;; li x29, ((0xffffffffff030303) & ((1 << (32 - 1) << 1) - 1)); li gp, 14; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff81818181) & ((1 << (32 - 1) << 1) - 1)); li x2, ((14) & ((1 << (32 - 1) << 1) - 1)); sra x30, x1, x2;; li x29, ((0xfffffffffffe0606) & ((1 << (32 - 1) << 1) - 1)); li gp, 15; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff81818181) & ((1 << (32 - 1) << 1) - 1)); li x2, ((31) & ((1 << (32 - 1) << 1) - 1)); sra x30, x1, x2;; li x29, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); li gp, 16; bne x30, x29, fail; call update_test;

  # Verify that shifts only use bottom five bits

  li x1, ((0xffffffff81818181) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xffffffffffffffc0) & ((1 << (32 - 1) << 1) - 1)); sra x30, x1, x2;; li x29, ((0xffffffff81818181) & ((1 << (32 - 1) << 1) - 1)); li gp, 17; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff81818181) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xffffffffffffffc1) & ((1 << (32 - 1) << 1) - 1)); sra x30, x1, x2;; li x29, ((0xffffffffc0c0c0c0) & ((1 << (32 - 1) << 1) - 1)); li gp, 18; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff81818181) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xffffffffffffffc7) & ((1 << (32 - 1) << 1) - 1)); sra x30, x1, x2;; li x29, ((0xffffffffff030303) & ((1 << (32 - 1) << 1) - 1)); li gp, 19; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff81818181) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xffffffffffffffce) & ((1 << (32 - 1) << 1) - 1)); sra x30, x1, x2;; li x29, ((0xfffffffffffe0606) & ((1 << (32 - 1) << 1) - 1)); li gp, 20; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff81818181) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); sra x30, x1, x2;; li x29, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); li gp, 21; bne x30, x29, fail; call update_test;
jr x31

slt_test:  li x10,0; sw x10, 0(x12)	#16 Tests

  #-------------------------------------------------------------
  # Arithmetic tests
  #-------------------------------------------------------------

  li x1, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); slt x30, x1, x2;; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 2; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000000000001) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0000000000000001) & ((1 << (32 - 1) << 1) - 1)); slt x30, x1, x2;; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 3; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000000000003) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0000000000000007) & ((1 << (32 - 1) << 1) - 1)); slt x30, x1, x2;; li x29, ((1) & ((1 << (32 - 1) << 1) - 1)); li gp, 4; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000000000007) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0000000000000003) & ((1 << (32 - 1) << 1) - 1)); slt x30, x1, x2;; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 5; bne x30, x29, fail; call update_test;

  li x1, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xffffffffffff8000) & ((1 << (32 - 1) << 1) - 1)); slt x30, x1, x2;; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 6; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); slt x30, x1, x2;; li x29, ((1) & ((1 << (32 - 1) << 1) - 1)); li gp, 7; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xffffffffffff8000) & ((1 << (32 - 1) << 1) - 1)); slt x30, x1, x2;; li x29, ((1) & ((1 << (32 - 1) << 1) - 1)); li gp, 8; bne x30, x29, fail; call update_test;

  li x1, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0000000000007fff) & ((1 << (32 - 1) << 1) - 1)); slt x30, x1, x2;; li x29, ((1) & ((1 << (32 - 1) << 1) - 1)); li gp, 9; bne x30, x29, fail; call update_test;
  li x1, ((0x000000007fffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); slt x30, x1, x2;; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 10; bne x30, x29, fail; call update_test;
  li x1, ((0x000000007fffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0000000000007fff) & ((1 << (32 - 1) << 1) - 1)); slt x30, x1, x2;; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 11; bne x30, x29, fail; call update_test;

  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0000000000007fff) & ((1 << (32 - 1) << 1) - 1)); slt x30, x1, x2;; li x29, ((1) & ((1 << (32 - 1) << 1) - 1)); li gp, 12; bne x30, x29, fail; call update_test;
  li x1, ((0x000000007fffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xffffffffffff8000) & ((1 << (32 - 1) << 1) - 1)); slt x30, x1, x2;; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 13; bne x30, x29, fail; call update_test;

  li x1, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); slt x30, x1, x2;; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 14; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x0000000000000001) & ((1 << (32 - 1) << 1) - 1)); slt x30, x1, x2;; li x29, ((1) & ((1 << (32 - 1) << 1) - 1)); li gp, 15; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); slt x30, x1, x2;; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 16; bne x30, x29, fail; call update_test;

  #-------------------------------------------------------------
  # Source/Destination tests
  #-------------------------------------------------------------

  li x1, ((14) & ((1 << (32 - 1) << 1) - 1)); li x2, ((13) & ((1 << (32 - 1) << 1) - 1)); slt x1, x1, x2;; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 17; bne x1, x29, fail; call update_test;
jr x31

sltu_test:  li x10,0; sw x10, 0(x12)		#16 Tests

  #-------------------------------------------------------------
  # Arithmetic tests
  #-------------------------------------------------------------

  li x1, ((0x00000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x00000000) & ((1 << (32 - 1) << 1) - 1)); sltu x30, x1, x2;; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 2; bne x30, x29, fail; call update_test;
  li x1, ((0x00000001) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x00000001) & ((1 << (32 - 1) << 1) - 1)); sltu x30, x1, x2;; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 3; bne x30, x29, fail; call update_test;
  li x1, ((0x00000003) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x00000007) & ((1 << (32 - 1) << 1) - 1)); sltu x30, x1, x2;; li x29, ((1) & ((1 << (32 - 1) << 1) - 1)); li gp, 4; bne x30, x29, fail; call update_test;
  li x1, ((0x00000007) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x00000003) & ((1 << (32 - 1) << 1) - 1)); sltu x30, x1, x2;; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 5; bne x30, x29, fail; call update_test;

  li x1, ((0x00000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xffff8000) & ((1 << (32 - 1) << 1) - 1)); sltu x30, x1, x2;; li x29, ((1) & ((1 << (32 - 1) << 1) - 1)); li gp, 6; bne x30, x29, fail; call update_test;
  li x1, ((0x80000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x00000000) & ((1 << (32 - 1) << 1) - 1)); sltu x30, x1, x2;; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 7; bne x30, x29, fail; call update_test;
  li x1, ((0x80000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xffff8000) & ((1 << (32 - 1) << 1) - 1)); sltu x30, x1, x2;; li x29, ((1) & ((1 << (32 - 1) << 1) - 1)); li gp, 8; bne x30, x29, fail; call update_test;

  li x1, ((0x00000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x00007fff) & ((1 << (32 - 1) << 1) - 1)); sltu x30, x1, x2;; li x29, ((1) & ((1 << (32 - 1) << 1) - 1)); li gp, 9; bne x30, x29, fail; call update_test;
  li x1, ((0x7fffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x00000000) & ((1 << (32 - 1) << 1) - 1)); sltu x30, x1, x2;; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 10; bne x30, x29, fail; call update_test;
  li x1, ((0x7fffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x00007fff) & ((1 << (32 - 1) << 1) - 1)); sltu x30, x1, x2;; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 11; bne x30, x29, fail; call update_test;

  li x1, ((0x80000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x00007fff) & ((1 << (32 - 1) << 1) - 1)); sltu x30, x1, x2;; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 12; bne x30, x29, fail; call update_test;
  li x1, ((0x7fffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xffff8000) & ((1 << (32 - 1) << 1) - 1)); sltu x30, x1, x2;; li x29, ((1) & ((1 << (32 - 1) << 1) - 1)); li gp, 13; bne x30, x29, fail; call update_test;

  li x1, ((0x00000000) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xffffffff) & ((1 << (32 - 1) << 1) - 1)); sltu x30, x1, x2;; li x29, ((1) & ((1 << (32 - 1) << 1) - 1)); li gp, 14; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0x00000001) & ((1 << (32 - 1) << 1) - 1)); sltu x30, x1, x2;; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 15; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff) & ((1 << (32 - 1) << 1) - 1)); li x2, ((0xffffffff) & ((1 << (32 - 1) << 1) - 1)); sltu x30, x1, x2;; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 16; bne x30, x29, fail; call update_test;

  #-------------------------------------------------------------
  # Source/Destination tests
  #-------------------------------------------------------------

  li x1, ((14) & ((1 << (32 - 1) << 1) - 1)); li x2, ((13) & ((1 << (32 - 1) << 1) - 1)); sltu x1, x1, x2;; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 17; bne x1, x29, fail; call update_test;
jr x31

addi_test:  li x10,0; sw x10, 0(x12)		#16 Tests

  #-------------------------------------------------------------
  # Arithmetic tests
  #-------------------------------------------------------------

  li x1, ((0x00000000) & ((1 << (32 - 1) << 1) - 1)); addi x30, x1, ((0x000) | (-(((0x000) >> 11) & 1) << 11));; li x29, ((0x00000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 2; bne x30, x29, fail; call update_test;
  li x1, ((0x00000001) & ((1 << (32 - 1) << 1) - 1)); addi x30, x1, ((0x001) | (-(((0x001) >> 11) & 1) << 11));; li x29, ((0x00000002) & ((1 << (32 - 1) << 1) - 1)); li gp, 3; bne x30, x29, fail; call update_test;
  li x1, ((0x00000003) & ((1 << (32 - 1) << 1) - 1)); addi x30, x1, ((0x007) | (-(((0x007) >> 11) & 1) << 11));; li x29, ((0x0000000a) & ((1 << (32 - 1) << 1) - 1)); li gp, 4; bne x30, x29, fail; call update_test;

  li x1, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); addi x30, x1, ((0x800) | (-(((0x800) >> 11) & 1) << 11));; li x29, ((0xfffffffffffff800) & ((1 << (32 - 1) << 1) - 1)); li gp, 5; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); addi x30, x1, ((0x000) | (-(((0x000) >> 11) & 1) << 11));; li x29, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 6; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); addi x30, x1, ((0x800) | (-(((0x800) >> 11) & 1) << 11));; li x29, ((0xffffffff7ffff800) & ((1 << (32 - 1) << 1) - 1)); li gp, 7; bne x30, x29, fail; call update_test;

  li x1, ((0x00000000) & ((1 << (32 - 1) << 1) - 1)); addi x30, x1, ((0x7ff) | (-(((0x7ff) >> 11) & 1) << 11));; li x29, ((0x00000000000007ff) & ((1 << (32 - 1) << 1) - 1)); li gp, 8; bne x30, x29, fail; call update_test;
  li x1, ((0x7fffffff) & ((1 << (32 - 1) << 1) - 1)); addi x30, x1, ((0x000) | (-(((0x000) >> 11) & 1) << 11));; li x29, ((0x000000007fffffff) & ((1 << (32 - 1) << 1) - 1)); li gp, 9; bne x30, x29, fail; call update_test;
  li x1, ((0x7fffffff) & ((1 << (32 - 1) << 1) - 1)); addi x30, x1, ((0x7ff) | (-(((0x7ff) >> 11) & 1) << 11));; li x29, ((0x00000000800007fe) & ((1 << (32 - 1) << 1) - 1)); li gp, 10; bne x30, x29, fail; call update_test;

  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); addi x30, x1, ((0x7ff) | (-(((0x7ff) >> 11) & 1) << 11));; li x29, ((0xffffffff800007ff) & ((1 << (32 - 1) << 1) - 1)); li gp, 11; bne x30, x29, fail; call update_test;
  li x1, ((0x000000007fffffff) & ((1 << (32 - 1) << 1) - 1)); addi x30, x1, ((0x800) | (-(((0x800) >> 11) & 1) << 11));; li x29, ((0x000000007ffff7ff) & ((1 << (32 - 1) << 1) - 1)); li gp, 12; bne x30, x29, fail; call update_test;

  li x1, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); addi x30, x1, ((0xfff) | (-(((0xfff) >> 11) & 1) << 11));; li x29, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); li gp, 13; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); addi x30, x1, ((0x001) | (-(((0x001) >> 11) & 1) << 11));; li x29, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 14; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); addi x30, x1, ((0xfff) | (-(((0xfff) >> 11) & 1) << 11));; li x29, ((0xfffffffffffffffe) & ((1 << (32 - 1) << 1) - 1)); li gp, 15; bne x30, x29, fail; call update_test;

  li x1, ((0x7fffffff) & ((1 << (32 - 1) << 1) - 1)); addi x30, x1, ((0x001) | (-(((0x001) >> 11) & 1) << 11));; li x29, ((0x0000000080000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 16; bne x30, x29, fail; call update_test;

  li x1, ((13) & ((1 << (32 - 1) << 1) - 1)); addi x1, x1, ((11) | (-(((11) >> 11) & 1) << 11));; li x29, ((24) & ((1 << (32 - 1) << 1) - 1)); li gp, 17; bne x1, x29, fail; call update_test;
jr x31

andi_test:  li x10,0; sw x10, 0(x12)		#5 tests

  #-------------------------------------------------------------
  # Logical tests
  #-------------------------------------------------------------

  li x1, ((0xff00ff00) & ((1 << (32 - 1) << 1) - 1)); andi x30, x1, ((0xf0f) | (-(((0xf0f) >> 11) & 1) << 11));; li x29, ((0xff00ff00) & ((1 << (32 - 1) << 1) - 1)); li gp, 2; bne x30, x29, fail; call update_test;
  li x1, ((0x0ff00ff0) & ((1 << (32 - 1) << 1) - 1)); andi x30, x1, ((0x0f0) | (-(((0x0f0) >> 11) & 1) << 11));; li x29, ((0x000000f0) & ((1 << (32 - 1) << 1) - 1)); li gp, 3; bne x30, x29, fail; call update_test;
  li x1, ((0x00ff00ff) & ((1 << (32 - 1) << 1) - 1)); andi x30, x1, ((0x70f) | (-(((0x70f) >> 11) & 1) << 11));; li x29, ((0x0000000f) & ((1 << (32 - 1) << 1) - 1)); li gp, 4; bne x30, x29, fail; call update_test;
  li x1, ((0xf00ff00f) & ((1 << (32 - 1) << 1) - 1)); andi x30, x1, ((0x0f0) | (-(((0x0f0) >> 11) & 1) << 11));; li x29, ((0x00000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 5; bne x30, x29, fail; call update_test;

  #-------------------------------------------------------------
  # Source/Destination tests
  #-------------------------------------------------------------

  li x1, ((0xff00ff00) & ((1 << (32 - 1) << 1) - 1)); andi x1, x1, ((0x0f0) | (-(((0x0f0) >> 11) & 1) << 11));; li x29, ((0x00000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 6; bne x1, x29, fail; call update_test;


jr x31

ori_test:  li x10,0; sw x10, 0(x12)	#5 tests

  #-------------------------------------------------------------
  # Logical tests
  #-------------------------------------------------------------

  li x1, ((0xffffffffff00ff00) & ((1 << (32 - 1) << 1) - 1)); ori x30, x1, ((0xf0f) | (-(((0xf0f) >> 11) & 1) << 11));; li x29, ((0xffffffffffffff0f) & ((1 << (32 - 1) << 1) - 1)); li gp, 2; bne x30, x29, fail; call update_test;
  li x1, ((0x000000000ff00ff0) & ((1 << (32 - 1) << 1) - 1)); ori x30, x1, ((0x0f0) | (-(((0x0f0) >> 11) & 1) << 11));; li x29, ((0x000000000ff00ff0) & ((1 << (32 - 1) << 1) - 1)); li gp, 3; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000000ff00ff) & ((1 << (32 - 1) << 1) - 1)); ori x30, x1, ((0x70f) | (-(((0x70f) >> 11) & 1) << 11));; li x29, ((0x0000000000ff07ff) & ((1 << (32 - 1) << 1) - 1)); li gp, 4; bne x30, x29, fail; call update_test;
  li x1, ((0xfffffffff00ff00f) & ((1 << (32 - 1) << 1) - 1)); ori x30, x1, ((0x0f0) | (-(((0x0f0) >> 11) & 1) << 11));; li x29, ((0xfffffffff00ff0ff) & ((1 << (32 - 1) << 1) - 1)); li gp, 5; bne x30, x29, fail; call update_test;

  #-------------------------------------------------------------
  # Source/Destination tests
  #-------------------------------------------------------------

  li x1, ((0xff00ff00) & ((1 << (32 - 1) << 1) - 1)); ori x1, x1, ((0x0f0) | (-(((0x0f0) >> 11) & 1) << 11));; li x29, ((0xff00fff0) & ((1 << (32 - 1) << 1) - 1)); li gp, 6; bne x1, x29, fail; call update_test;

jr x31

xori_test:  li x10,0; sw x10, 0(x12)		#5 tests

  #-------------------------------------------------------------
  # Logical tests
  #-------------------------------------------------------------

  li x1, ((0x0000000000ff0f00) & ((1 << (32 - 1) << 1) - 1)); xori x30, x1, ((0xf0f) | (-(((0xf0f) >> 11) & 1) << 11));; li x29, ((0xffffffffff00f00f) & ((1 << (32 - 1) << 1) - 1)); li gp, 2; bne x30, x29, fail; call update_test;
  li x1, ((0x000000000ff00ff0) & ((1 << (32 - 1) << 1) - 1)); xori x30, x1, ((0x0f0) | (-(((0x0f0) >> 11) & 1) << 11));; li x29, ((0x000000000ff00f00) & ((1 << (32 - 1) << 1) - 1)); li gp, 3; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000000ff08ff) & ((1 << (32 - 1) << 1) - 1)); xori x30, x1, ((0x70f) | (-(((0x70f) >> 11) & 1) << 11));; li x29, ((0x0000000000ff0ff0) & ((1 << (32 - 1) << 1) - 1)); li gp, 4; bne x30, x29, fail; call update_test;
  li x1, ((0xfffffffff00ff00f) & ((1 << (32 - 1) << 1) - 1)); xori x30, x1, ((0x0f0) | (-(((0x0f0) >> 11) & 1) << 11));; li x29, ((0xfffffffff00ff0ff) & ((1 << (32 - 1) << 1) - 1)); li gp, 5; bne x30, x29, fail; call update_test;

  #-------------------------------------------------------------
  # Source/Destination tests
  #-------------------------------------------------------------

  li x1, ((0xffffffffff00f700) & ((1 << (32 - 1) << 1) - 1)); xori x1, x1, ((0x70f) | (-(((0x70f) >> 11) & 1) << 11));; li x29, ((0xffffffffff00f00f) & ((1 << (32 - 1) << 1) - 1)); li gp, 6; bne x1, x29, fail; call update_test;

jr x31

slli_test:  li x10,0; sw x10, 0(x12)	#15 tests

  #-------------------------------------------------------------
  # Arithmetic tests
  #-------------------------------------------------------------

  li x1, ((0x0000000000000001) & ((1 << (32 - 1) << 1) - 1)); slli x30, x1, ((0) | (-(((0) >> 11) & 1) << 11));; li x29, ((0x0000000000000001) & ((1 << (32 - 1) << 1) - 1)); li gp, 2; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000000000001) & ((1 << (32 - 1) << 1) - 1)); slli x30, x1, ((1) | (-(((1) >> 11) & 1) << 11));; li x29, ((0x0000000000000002) & ((1 << (32 - 1) << 1) - 1)); li gp, 3; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000000000001) & ((1 << (32 - 1) << 1) - 1)); slli x30, x1, ((7) | (-(((7) >> 11) & 1) << 11));; li x29, ((0x0000000000000080) & ((1 << (32 - 1) << 1) - 1)); li gp, 4; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000000000001) & ((1 << (32 - 1) << 1) - 1)); slli x30, x1, ((14) | (-(((14) >> 11) & 1) << 11));; li x29, ((0x0000000000004000) & ((1 << (32 - 1) << 1) - 1)); li gp, 5; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000000000001) & ((1 << (32 - 1) << 1) - 1)); slli x30, x1, ((31) | (-(((31) >> 11) & 1) << 11));; li x29, ((0x0000000080000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 6; bne x30, x29, fail; call update_test;

  li x1, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); slli x30, x1, ((0) | (-(((0) >> 11) & 1) << 11));; li x29, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); li gp, 7; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); slli x30, x1, ((1) | (-(((1) >> 11) & 1) << 11));; li x29, ((0xfffffffffffffffe) & ((1 << (32 - 1) << 1) - 1)); li gp, 8; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); slli x30, x1, ((7) | (-(((7) >> 11) & 1) << 11));; li x29, ((0xffffffffffffff80) & ((1 << (32 - 1) << 1) - 1)); li gp, 9; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); slli x30, x1, ((14) | (-(((14) >> 11) & 1) << 11));; li x29, ((0xffffffffffffc000) & ((1 << (32 - 1) << 1) - 1)); li gp, 10; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); slli x30, x1, ((31) | (-(((31) >> 11) & 1) << 11));; li x29, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 11; bne x30, x29, fail; call update_test;

  li x1, ((0x0000000021212121) & ((1 << (32 - 1) << 1) - 1)); slli x30, x1, ((0) | (-(((0) >> 11) & 1) << 11));; li x29, ((0x0000000021212121) & ((1 << (32 - 1) << 1) - 1)); li gp, 12; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000021212121) & ((1 << (32 - 1) << 1) - 1)); slli x30, x1, ((1) | (-(((1) >> 11) & 1) << 11));; li x29, ((0x0000000042424242) & ((1 << (32 - 1) << 1) - 1)); li gp, 13; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000021212121) & ((1 << (32 - 1) << 1) - 1)); slli x30, x1, ((7) | (-(((7) >> 11) & 1) << 11));; li x29, ((0x0000001090909080) & ((1 << (32 - 1) << 1) - 1)); li gp, 14; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000021212121) & ((1 << (32 - 1) << 1) - 1)); slli x30, x1, ((14) | (-(((14) >> 11) & 1) << 11));; li x29, ((0x0000084848484000) & ((1 << (32 - 1) << 1) - 1)); li gp, 15; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000021212121) & ((1 << (32 - 1) << 1) - 1)); slli x30, x1, ((31) | (-(((31) >> 11) & 1) << 11));; li x29, ((0x1090909080000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 16; bne x30, x29, fail; call update_test;

jr x31

srli_test:  li x10,0; sw x10, 0(x12)	#16 tests

  #-------------------------------------------------------------
  # Arithmetic tests
  #-------------------------------------------------------------
  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); srli x30, x1, ((0) | (-(((0) >> 11) & 1) << 11));; li x29, ((((0xffffffff80000000) & ((1 << (32 -1) << 1) - 1)) >> (0)) & ((1 << (32 - 1) << 1) - 1)); li gp, 2; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); srli x30, x1, ((1) | (-(((1) >> 11) & 1) << 11));; li x29, ((((0xffffffff80000000) & ((1 << (32 -1) << 1) - 1)) >> (1)) & ((1 << (32 - 1) << 1) - 1)); li gp, 3; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); srli x30, x1, ((7) | (-(((7) >> 11) & 1) << 11));; li x29, ((((0xffffffff80000000) & ((1 << (32 -1) << 1) - 1)) >> (7)) & ((1 << (32 - 1) << 1) - 1)); li gp, 4; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); srli x30, x1, ((14) | (-(((14) >> 11) & 1) << 11));; li x29, ((((0xffffffff80000000) & ((1 << (32 -1) << 1) - 1)) >> (14)) & ((1 << (32 - 1) << 1) - 1)); li gp, 5; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff80000001) & ((1 << (32 - 1) << 1) - 1)); srli x30, x1, ((31) | (-(((31) >> 11) & 1) << 11));; li x29, ((((0xffffffff80000001) & ((1 << (32 -1) << 1) - 1)) >> (31)) & ((1 << (32 - 1) << 1) - 1)); li gp, 6; bne x30, x29, fail; call update_test;

  li x1, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); srli x30, x1, ((0) | (-(((0) >> 11) & 1) << 11));; li x29, ((((0xffffffffffffffff) & ((1 << (32 -1) << 1) - 1)) >> (0)) & ((1 << (32 - 1) << 1) - 1)); li gp, 7; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); srli x30, x1, ((1) | (-(((1) >> 11) & 1) << 11));; li x29, ((((0xffffffffffffffff) & ((1 << (32 -1) << 1) - 1)) >> (1)) & ((1 << (32 - 1) << 1) - 1)); li gp, 8; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); srli x30, x1, ((7) | (-(((7) >> 11) & 1) << 11));; li x29, ((((0xffffffffffffffff) & ((1 << (32 -1) << 1) - 1)) >> (7)) & ((1 << (32 - 1) << 1) - 1)); li gp, 9; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); srli x30, x1, ((14) | (-(((14) >> 11) & 1) << 11));; li x29, ((((0xffffffffffffffff) & ((1 << (32 -1) << 1) - 1)) >> (14)) & ((1 << (32 - 1) << 1) - 1)); li gp, 10; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); srli x30, x1, ((31) | (-(((31) >> 11) & 1) << 11));; li x29, ((((0xffffffffffffffff) & ((1 << (32 -1) << 1) - 1)) >> (31)) & ((1 << (32 - 1) << 1) - 1)); li gp, 11; bne x30, x29, fail; call update_test;

  li x1, ((0x0000000021212121) & ((1 << (32 - 1) << 1) - 1)); srli x30, x1, ((0) | (-(((0) >> 11) & 1) << 11));; li x29, ((((0x0000000021212121) & ((1 << (32 -1) << 1) - 1)) >> (0)) & ((1 << (32 - 1) << 1) - 1)); li gp, 12; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000021212121) & ((1 << (32 - 1) << 1) - 1)); srli x30, x1, ((1) | (-(((1) >> 11) & 1) << 11));; li x29, ((((0x0000000021212121) & ((1 << (32 -1) << 1) - 1)) >> (1)) & ((1 << (32 - 1) << 1) - 1)); li gp, 13; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000021212121) & ((1 << (32 - 1) << 1) - 1)); srli x30, x1, ((7) | (-(((7) >> 11) & 1) << 11));; li x29, ((((0x0000000021212121) & ((1 << (32 -1) << 1) - 1)) >> (7)) & ((1 << (32 - 1) << 1) - 1)); li gp, 14; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000021212121) & ((1 << (32 - 1) << 1) - 1)); srli x30, x1, ((14) | (-(((14) >> 11) & 1) << 11));; li x29, ((((0x0000000021212121) & ((1 << (32 -1) << 1) - 1)) >> (14)) & ((1 << (32 - 1) << 1) - 1)); li gp, 15; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000021212121) & ((1 << (32 - 1) << 1) - 1)); srli x30, x1, ((31) | (-(((31) >> 11) & 1) << 11));; li x29, ((((0x0000000021212121) & ((1 << (32 -1) << 1) - 1)) >> (31)) & ((1 << (32 - 1) << 1) - 1)); li gp, 16; bne x30, x29, fail; call update_test;

  #-------------------------------------------------------------
  # Source/Destination tests
  #-------------------------------------------------------------

  li x1, ((0x80000000) & ((1 << (32 - 1) << 1) - 1)); srli x1, x1, ((7) | (-(((7) >> 11) & 1) << 11));; li x29, ((0x01000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 17; bne x1, x29, fail; call update_test;
jr x31

srai_test:  li x10,0; sw x10, 0(x12)	#16 tests

  #-------------------------------------------------------------
  # Arithmetic tests
  #-------------------------------------------------------------

  li x1, ((0xffffff8000000000) & ((1 << (32 - 1) << 1) - 1)); srai x30, x1, ((0) | (-(((0) >> 11) & 1) << 11));; li x29, ((0xffffff8000000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 2; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); srai x30, x1, ((1) | (-(((1) >> 11) & 1) << 11));; li x29, ((0xffffffffc0000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 3; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); srai x30, x1, ((7) | (-(((7) >> 11) & 1) << 11));; li x29, ((0xffffffffff000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 4; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); srai x30, x1, ((14) | (-(((14) >> 11) & 1) << 11));; li x29, ((0xfffffffffffe0000) & ((1 << (32 - 1) << 1) - 1)); li gp, 5; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff80000001) & ((1 << (32 - 1) << 1) - 1)); srai x30, x1, ((31) | (-(((31) >> 11) & 1) << 11));; li x29, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); li gp, 6; bne x30, x29, fail; call update_test;

  li x1, ((0x000000007fffffff) & ((1 << (32 - 1) << 1) - 1)); srai x30, x1, ((0) | (-(((0) >> 11) & 1) << 11));; li x29, ((0x000000007fffffff) & ((1 << (32 - 1) << 1) - 1)); li gp, 7; bne x30, x29, fail; call update_test;
  li x1, ((0x000000007fffffff) & ((1 << (32 - 1) << 1) - 1)); srai x30, x1, ((1) | (-(((1) >> 11) & 1) << 11));; li x29, ((0x000000003fffffff) & ((1 << (32 - 1) << 1) - 1)); li gp, 8; bne x30, x29, fail; call update_test;
  li x1, ((0x000000007fffffff) & ((1 << (32 - 1) << 1) - 1)); srai x30, x1, ((7) | (-(((7) >> 11) & 1) << 11));; li x29, ((0x0000000000ffffff) & ((1 << (32 - 1) << 1) - 1)); li gp, 9; bne x30, x29, fail; call update_test;
  li x1, ((0x000000007fffffff) & ((1 << (32 - 1) << 1) - 1)); srai x30, x1, ((14) | (-(((14) >> 11) & 1) << 11));; li x29, ((0x000000000001ffff) & ((1 << (32 - 1) << 1) - 1)); li gp, 10; bne x30, x29, fail; call update_test;
  li x1, ((0x000000007fffffff) & ((1 << (32 - 1) << 1) - 1)); srai x30, x1, ((31) | (-(((31) >> 11) & 1) << 11));; li x29, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 11; bne x30, x29, fail; call update_test;

  li x1, ((0xffffffff81818181) & ((1 << (32 - 1) << 1) - 1)); srai x30, x1, ((0) | (-(((0) >> 11) & 1) << 11));; li x29, ((0xffffffff81818181) & ((1 << (32 - 1) << 1) - 1)); li gp, 12; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff81818181) & ((1 << (32 - 1) << 1) - 1)); srai x30, x1, ((1) | (-(((1) >> 11) & 1) << 11));; li x29, ((0xffffffffc0c0c0c0) & ((1 << (32 - 1) << 1) - 1)); li gp, 13; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff81818181) & ((1 << (32 - 1) << 1) - 1)); srai x30, x1, ((7) | (-(((7) >> 11) & 1) << 11));; li x29, ((0xffffffffff030303) & ((1 << (32 - 1) << 1) - 1)); li gp, 14; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff81818181) & ((1 << (32 - 1) << 1) - 1)); srai x30, x1, ((14) | (-(((14) >> 11) & 1) << 11));; li x29, ((0xfffffffffffe0606) & ((1 << (32 - 1) << 1) - 1)); li gp, 15; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff81818181) & ((1 << (32 - 1) << 1) - 1)); srai x30, x1, ((31) | (-(((31) >> 11) & 1) << 11));; li x29, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); li gp, 16; bne x30, x29, fail; call update_test;

  #-------------------------------------------------------------
  # Source/Destination tests
  #-------------------------------------------------------------

  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); srai x1, x1, ((7) | (-(((7) >> 11) & 1) << 11));; li x29, ((0xffffffffff000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 17; bne x1, x29, fail; call update_test;

jr x31

sltiu_test:  li x10,0; sw x10, 0(x12)	#16 tests

  #-------------------------------------------------------------
  # Arithmetic tests
  #-------------------------------------------------------------

  li x1, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); sltiu x30, x1, ((0x000) | (-(((0x000) >> 11) & 1) << 11));; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 2; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000000000001) & ((1 << (32 - 1) << 1) - 1)); sltiu x30, x1, ((0x001) | (-(((0x001) >> 11) & 1) << 11));; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 3; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000000000003) & ((1 << (32 - 1) << 1) - 1)); sltiu x30, x1, ((0x007) | (-(((0x007) >> 11) & 1) << 11));; li x29, ((1) & ((1 << (32 - 1) << 1) - 1)); li gp, 4; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000000000007) & ((1 << (32 - 1) << 1) - 1)); sltiu x30, x1, ((0x003) | (-(((0x003) >> 11) & 1) << 11));; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 5; bne x30, x29, fail; call update_test;

  li x1, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); sltiu x30, x1, ((0x800) | (-(((0x800) >> 11) & 1) << 11));; li x29, ((1) & ((1 << (32 - 1) << 1) - 1)); li gp, 6; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); sltiu x30, x1, ((0x000) | (-(((0x000) >> 11) & 1) << 11));; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 7; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); sltiu x30, x1, ((0x800) | (-(((0x800) >> 11) & 1) << 11));; li x29, ((1) & ((1 << (32 - 1) << 1) - 1)); li gp, 8; bne x30, x29, fail; call update_test;

  li x1, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); sltiu x30, x1, ((0x7ff) | (-(((0x7ff) >> 11) & 1) << 11));; li x29, ((1) & ((1 << (32 - 1) << 1) - 1)); li gp, 9; bne x30, x29, fail; call update_test;
  li x1, ((0x000000007fffffff) & ((1 << (32 - 1) << 1) - 1)); sltiu x30, x1, ((0x000) | (-(((0x000) >> 11) & 1) << 11));; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 10; bne x30, x29, fail; call update_test;
  li x1, ((0x000000007fffffff) & ((1 << (32 - 1) << 1) - 1)); sltiu x30, x1, ((0x7ff) | (-(((0x7ff) >> 11) & 1) << 11));; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 11; bne x30, x29, fail; call update_test;

  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); sltiu x30, x1, ((0x7ff) | (-(((0x7ff) >> 11) & 1) << 11));; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 12; bne x30, x29, fail; call update_test;
  li x1, ((0x000000007fffffff) & ((1 << (32 - 1) << 1) - 1)); sltiu x30, x1, ((0x800) | (-(((0x800) >> 11) & 1) << 11));; li x29, ((1) & ((1 << (32 - 1) << 1) - 1)); li gp, 13; bne x30, x29, fail; call update_test;

  li x1, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); sltiu x30, x1, ((0xfff) | (-(((0xfff) >> 11) & 1) << 11));; li x29, ((1) & ((1 << (32 - 1) << 1) - 1)); li gp, 14; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); sltiu x30, x1, ((0x001) | (-(((0x001) >> 11) & 1) << 11));; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 15; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); sltiu x30, x1, ((0xfff) | (-(((0xfff) >> 11) & 1) << 11));; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 16; bne x30, x29, fail; call update_test;

  #-------------------------------------------------------------
  # Source/Destination tests
  #-------------------------------------------------------------

  li x1, ((11) & ((1 << (32 - 1) << 1) - 1)); sltiu x1, x1, ((13) | (-(((13) >> 11) & 1) << 11));; li x29, ((1) & ((1 << (32 - 1) << 1) - 1)); li gp, 17; bne x1, x29, fail; call update_test;
jr x31

slti_test:  li x10,0; sw x10, 0(x12)	#16 tests

  #-------------------------------------------------------------
  # Arithmetic tests
  #-------------------------------------------------------------

  li x1, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); slti x30, x1, ((0x000) | (-(((0x000) >> 11) & 1) << 11));; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 2; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000000000001) & ((1 << (32 - 1) << 1) - 1)); slti x30, x1, ((0x001) | (-(((0x001) >> 11) & 1) << 11));; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 3; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000000000003) & ((1 << (32 - 1) << 1) - 1)); slti x30, x1, ((0x007) | (-(((0x007) >> 11) & 1) << 11));; li x29, ((1) & ((1 << (32 - 1) << 1) - 1)); li gp, 4; bne x30, x29, fail; call update_test;
  li x1, ((0x0000000000000007) & ((1 << (32 - 1) << 1) - 1)); slti x30, x1, ((0x003) | (-(((0x003) >> 11) & 1) << 11));; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 5; bne x30, x29, fail; call update_test;

  li x1, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); slti x30, x1, ((0x800) | (-(((0x800) >> 11) & 1) << 11));; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 6; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); slti x30, x1, ((0x000) | (-(((0x000) >> 11) & 1) << 11));; li x29, ((1) & ((1 << (32 - 1) << 1) - 1)); li gp, 7; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); slti x30, x1, ((0x800) | (-(((0x800) >> 11) & 1) << 11));; li x29, ((1) & ((1 << (32 - 1) << 1) - 1)); li gp, 8; bne x30, x29, fail; call update_test;

  li x1, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); slti x30, x1, ((0x7ff) | (-(((0x7ff) >> 11) & 1) << 11));; li x29, ((1) & ((1 << (32 - 1) << 1) - 1)); li gp, 9; bne x30, x29, fail; call update_test;
  li x1, ((0x000000007fffffff) & ((1 << (32 - 1) << 1) - 1)); slti x30, x1, ((0x000) | (-(((0x000) >> 11) & 1) << 11));; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 10; bne x30, x29, fail; call update_test;
  li x1, ((0x000000007fffffff) & ((1 << (32 - 1) << 1) - 1)); slti x30, x1, ((0x7ff) | (-(((0x7ff) >> 11) & 1) << 11));; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 11; bne x30, x29, fail; call update_test;

  li x1, ((0xffffffff80000000) & ((1 << (32 - 1) << 1) - 1)); slti x30, x1, ((0x7ff) | (-(((0x7ff) >> 11) & 1) << 11));; li x29, ((1) & ((1 << (32 - 1) << 1) - 1)); li gp, 12; bne x30, x29, fail; call update_test;
  li x1, ((0x000000007fffffff) & ((1 << (32 - 1) << 1) - 1)); slti x30, x1, ((0x800) | (-(((0x800) >> 11) & 1) << 11));; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 13; bne x30, x29, fail; call update_test;

  li x1, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); slti x30, x1, ((0xfff) | (-(((0xfff) >> 11) & 1) << 11));; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 14; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); slti x30, x1, ((0x001) | (-(((0x001) >> 11) & 1) << 11));; li x29, ((1) & ((1 << (32 - 1) << 1) - 1)); li gp, 15; bne x30, x29, fail; call update_test;
  li x1, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); slti x30, x1, ((0xfff) | (-(((0xfff) >> 11) & 1) << 11));; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 16; bne x30, x29, fail; call update_test;

  #-------------------------------------------------------------
  # Source/Destination tests
  #-------------------------------------------------------------

  li x1, ((11) & ((1 << (32 - 1) << 1) - 1)); slti x1, x1, ((13) | (-(((13) >> 11) & 1) << 11));; li x29, ((1) & ((1 << (32 - 1) << 1) - 1)); li gp, 17; bne x1, x29, fail; call update_test;
jr x31

lw_test:  li x10,0; sw x10, 0(x12)	# 9 tests

  #-------------------------------------------------------------
  # Basic tests
  #-------------------------------------------------------------

  la x1, tdatlw; lw x30, 0(x1);; li x29, ((0x0000000000ff00ff) & ((1 << (32 - 1) << 1) - 1)); li gp, 2; bne x30, x29, fail; call update_test;
  la x1, tdatlw; lw x30, 4(x1);; li x29, ((0xffffffffff00ff00) & ((1 << (32 - 1) << 1) - 1)); li gp, 3; bne x30, x29, fail; call update_test;
  la x1, tdatlw; lw x30, 8(x1);; li x29, ((0x000000000ff00ff0) & ((1 << (32 - 1) << 1) - 1)); li gp, 4; bne x30, x29, fail; call update_test;
  la x1, tdatlw; lw x30, 12(x1);; li x29, ((0xfffffffff00ff00f) & ((1 << (32 - 1) << 1) - 1)); li gp, 5; bne x30, x29, fail; call update_test;

  # Test with negative offset

  la x1, tdat4lw; lw x30, -12(x1);; li x29, ((0x0000000000ff00ff) & ((1 << (32 - 1) << 1) - 1)); li gp, 6; bne x30, x29, fail; call update_test;
  la x1, tdat4lw; lw x30, -8(x1);; li x29, ((0xffffffffff00ff00) & ((1 << (32 - 1) << 1) - 1)); li gp, 7; bne x30, x29, fail; call update_test;
  la x1, tdat4lw; lw x30, -4(x1);; li x29, ((0x000000000ff00ff0) & ((1 << (32 - 1) << 1) - 1)); li gp, 8; bne x30, x29, fail; call update_test;
  la x1, tdat4lw; lw x30, 0(x1);; li x29, ((0xfffffffff00ff00f) & ((1 << (32 - 1) << 1) - 1)); li gp, 9; bne x30, x29, fail; call update_test;

  # Test with a negative base

  la x1, tdatlw; addi x1, x1, -32; lw x5, 32(x1);; li x29, ((0x0000000000ff00ff) & ((1 << (32 - 1) << 1) - 1)); li gp, 10; bne x5, x29, fail; call update_test;


jr x31

sw_test:  li x10,0; sw x10, 0(x12)		# 9 tests

  #-------------------------------------------------------------
  # Basic tests
  #-------------------------------------------------------------

  la x1, tdat; li x2, 0x0000000000aa00aa; sw x2, 0(x1); lw x30, 0(x1);; li x29, ((0x0000000000aa00aa) & ((1 << (32 - 1) << 1) - 1)); li gp, 2; bne x30, x29, fail; call update_test;
  la x1, tdat; li x2, 0xffffffffaa00aa00; sw x2, 4(x1); lw x30, 4(x1);; li x29, ((0xffffffffaa00aa00) & ((1 << (32 - 1) << 1) - 1)); li gp, 3; bne x30, x29, fail; call update_test;
  la x1, tdat; li x2, 0x000000000aa00aa0; sw x2, 8(x1); lw x30, 8(x1);; li x29, ((0x000000000aa00aa0) & ((1 << (32 - 1) << 1) - 1)); li gp, 4; bne x30, x29, fail; call update_test;
  la x1, tdat; li x2, 0xffffffffa00aa00a; sw x2, 12(x1); lw x30, 12(x1);; li x29, ((0xffffffffa00aa00a) & ((1 << (32 - 1) << 1) - 1)); li gp, 5; bne x30, x29, fail; call update_test;

  # Test with negative offset

  la x1, tdat8; li x2, 0x0000000000aa00aa; sw x2, -12(x1); lw x30, -12(x1);; li x29, ((0x0000000000aa00aa) & ((1 << (32 - 1) << 1) - 1)); li gp, 6; bne x30, x29, fail; call update_test;
  la x1, tdat8; li x2, 0xffffffffaa00aa00; sw x2, -8(x1); lw x30, -8(x1);; li x29, ((0xffffffffaa00aa00) & ((1 << (32 - 1) << 1) - 1)); li gp, 7; bne x30, x29, fail; call update_test;
  la x1, tdat8; li x2, 0x000000000aa00aa0; sw x2, -4(x1); lw x30, -4(x1);; li x29, ((0x000000000aa00aa0) & ((1 << (32 - 1) << 1) - 1)); li gp, 8; bne x30, x29, fail; call update_test;
  la x1, tdat8; li x2, 0xffffffffa00aa00a; sw x2, 0(x1); lw x30, 0(x1);; li x29, ((0xffffffffa00aa00a) & ((1 << (32 - 1) << 1) - 1)); li gp, 9; bne x30, x29, fail; call update_test;

  # Test with a negative base

  la x1, tdat9; li x2, 0x12345678; addi x4, x1, -32; sw x2, 32(x4); lw x5, 0(x1);; li x29, ((0x12345678) & ((1 << (32 - 1) << 1) - 1)); li gp, 10; bne x5, x29, fail; call update_test;

jr x31

beq_test:  li x10,0; sw x10, 0(x12)	#7 tests

  #-------------------------------------------------------------
  # Branch tests
  #-------------------------------------------------------------

  # Each test checks both forward and backward branches

  li gp, 2; li x1, 0; li x2, 0; beq x1, x2, 2f; bne x0, gp, fail; 1: bne x0, gp, 3f; 2: beq x1, x2, 1b; bne x0, gp, fail; 3: call update_test;
  li gp, 3; li x1, 1; li x2, 1; beq x1, x2, 2f; bne x0, gp, fail; 1: bne x0, gp, 3f; 2: beq x1, x2, 1b; bne x0, gp, fail; 3: call update_test;
  li gp, 4; li x1, -1; li x2, -1; beq x1, x2, 2f; bne x0, gp, fail; 1: bne x0, gp, 3f; 2: beq x1, x2, 1b; bne x0, gp, fail; 3: call update_test;

  li gp, 5; li x1, 0; li x2, 1; beq x1, x2, 1f; bne x0, gp, 2f; 1: bne x0, gp, fail; 2: beq x1, x2, 1b; 3: call update_test;
  li gp, 6; li x1, 1; li x2, 0; beq x1, x2, 1f; bne x0, gp, 2f; 1: bne x0, gp, fail; 2: beq x1, x2, 1b; 3: call update_test;
  li gp, 7; li x1, -1; li x2, 1; beq x1, x2, 1f; bne x0, gp, 2f; 1: bne x0, gp, fail; 2: beq x1, x2, 1b; 3: call update_test;
  li gp, 8; li x1, 1; li x2, -1; beq x1, x2, 1f; bne x0, gp, 2f; 1: bne x0, gp, fail; 2: beq x1, x2, 1b; 3: call update_test;


jr x31

bne_test:  li x10,0; sw x10, 0(x12)	#7 tests

  #-------------------------------------------------------------
  # Branch tests
  #-------------------------------------------------------------

  # Each test checks both forward and backward branches

  li gp, 2; li x1, 0; li x2, 1; bne x1, x2, 2f; bne x0, gp, fail; 1: bne x0, gp, 3f; 2: bne x1, x2, 1b; bne x0, gp, fail; 3: call update_test;
  li gp, 3; li x1, 1; li x2, 0; bne x1, x2, 2f; bne x0, gp, fail; 1: bne x0, gp, 3f; 2: bne x1, x2, 1b; bne x0, gp, fail; 3: call update_test;
  li gp, 4; li x1, -1; li x2, 1; bne x1, x2, 2f; bne x0, gp, fail; 1: bne x0, gp, 3f; 2: bne x1, x2, 1b; bne x0, gp, fail; 3: call update_test;
  li gp, 5; li x1, 1; li x2, -1; bne x1, x2, 2f; bne x0, gp, fail; 1: bne x0, gp, 3f; 2: bne x1, x2, 1b; bne x0, gp, fail; 3: call update_test;

  li gp, 6; li x1, 0; li x2, 0; bne x1, x2, 1f; bne x0, gp, 2f; 1: bne x0, gp, fail; 2: bne x1, x2, 1b; 3: call update_test;
  li gp, 7; li x1, 1; li x2, 1; bne x1, x2, 1f; bne x0, gp, 2f; 1: bne x0, gp, fail; 2: bne x1, x2, 1b; 3: call update_test;
  li gp, 8; li x1, -1; li x2, -1; bne x1, x2, 1f; bne x0, gp, 2f; 1: bne x0, gp, fail; 2: bne x1, x2, 1b; 3: call update_test;

jr x31

blt_test:  li x10,0; sw x10, 0(x12)	#7 tests

  #-------------------------------------------------------------
  # Branch tests
  #-------------------------------------------------------------

  # Each test checks both forward and backward branches

  li gp, 2; li x1, 0; li x2, 1; blt x1, x2, 2f; bne x0, gp, fail; 1: bne x0, gp, 3f; 2: blt x1, x2, 1b; bne x0, gp, fail; 3: call update_test;
  li gp, 3; li x1, -1; li x2, 1; blt x1, x2, 2f; bne x0, gp, fail; 1: bne x0, gp, 3f; 2: blt x1, x2, 1b; bne x0, gp, fail; 3: call update_test;
  li gp, 4; li x1, -2; li x2, -1; blt x1, x2, 2f; bne x0, gp, fail; 1: bne x0, gp, 3f; 2: blt x1, x2, 1b; bne x0, gp, fail; 3: call update_test;

  li gp, 5; li x1, 1; li x2, 0; blt x1, x2, 1f; bne x0, gp, 2f; 1: bne x0, gp, fail; 2: blt x1, x2, 1b; 3: call update_test;
  li gp, 6; li x1, 1; li x2, -1; blt x1, x2, 1f; bne x0, gp, 2f; 1: bne x0, gp, fail; 2: blt x1, x2, 1b; 3: call update_test;
  li gp, 7; li x1, -1; li x2, -2; blt x1, x2, 1f; bne x0, gp, 2f; 1: bne x0, gp, fail; 2: blt x1, x2, 1b; 3: call update_test;
  li gp, 8; li x1, 1; li x2, -2; blt x1, x2, 1f; bne x0, gp, 2f; 1: bne x0, gp, fail; 2: blt x1, x2, 1b; 3: call update_test;

jr x31

bge_test:  li x10,0; sw x10, 0(x12)	#10 tests


  #-------------------------------------------------------------
  # Branch tests
  #-------------------------------------------------------------

  # Each test checks both forward and backward branches

  li gp, 2; li x1, 0; li x2, 0; bge x1, x2, 2f; bne x0, gp, fail; 1: bne x0, gp, 3f; 2: bge x1, x2, 1b; bne x0, gp, fail; 3: call update_test;
  li gp, 3; li x1, 1; li x2, 1; bge x1, x2, 2f; bne x0, gp, fail; 1: bne x0, gp, 3f; 2: bge x1, x2, 1b; bne x0, gp, fail; 3: call update_test;
  li gp, 4; li x1, -1; li x2, -1; bge x1, x2, 2f; bne x0, gp, fail; 1: bne x0, gp, 3f; 2: bge x1, x2, 1b; bne x0, gp, fail; 3: call update_test;
  li gp, 5; li x1, 1; li x2, 0; bge x1, x2, 2f; bne x0, gp, fail; 1: bne x0, gp, 3f; 2: bge x1, x2, 1b; bne x0, gp, fail; 3: call update_test;
  li gp, 6; li x1, 1; li x2, -1; bge x1, x2, 2f; bne x0, gp, fail; 1: bne x0, gp, 3f; 2: bge x1, x2, 1b; bne x0, gp, fail; 3: call update_test;
  li gp, 7; li x1, -1; li x2, -2; bge x1, x2, 2f; bne x0, gp, fail; 1: bne x0, gp, 3f; 2: bge x1, x2, 1b; bne x0, gp, fail; 3: call update_test;

  li gp, 8; li x1, 0; li x2, 1; bge x1, x2, 1f; bne x0, gp, 2f; 1: bne x0, gp, fail; 2: bge x1, x2, 1b; 3: call update_test;
  li gp, 9; li x1, -1; li x2, 1; bge x1, x2, 1f; bne x0, gp, 2f; 1: bne x0, gp, fail; 2: bge x1, x2, 1b; 3: call update_test;
  li gp, 10; li x1, -2; li x2, -1; bge x1, x2, 1f; bne x0, gp, 2f; 1: bne x0, gp, fail; 2: bge x1, x2, 1b; 3: call update_test;
  li gp, 11; li x1, -2; li x2, 1; bge x1, x2, 1f; bne x0, gp, 2f; 1: bne x0, gp, fail; 2: bge x1, x2, 1b; 3: call update_test;
  
jr x31

bltu_test:  li x10,0; sw x10, 0(x12)	#7 tests


  #-------------------------------------------------------------
  # Branch tests
  #-------------------------------------------------------------

  # Each test checks both forward and backward branches

  li gp, 2; li x1, 0x00000000; li x2, 0x00000001; bltu x1, x2, 2f; bne x0, gp, fail; 1: bne x0, gp, 3f; 2: bltu x1, x2, 1b; bne x0, gp, fail; 3: call update_test;
  li gp, 3; li x1, 0xfffffffe; li x2, 0xffffffff; bltu x1, x2, 2f; bne x0, gp, fail; 1: bne x0, gp, 3f; 2: bltu x1, x2, 1b; bne x0, gp, fail; 3: call update_test;
  li gp, 4; li x1, 0x00000000; li x2, 0xffffffff; bltu x1, x2, 2f; bne x0, gp, fail; 1: bne x0, gp, 3f; 2: bltu x1, x2, 1b; bne x0, gp, fail; 3: call update_test;

  li gp, 5; li x1, 0x00000001; li x2, 0x00000000; bltu x1, x2, 1f; bne x0, gp, 2f; 1: bne x0, gp, fail; 2: bltu x1, x2, 1b; 3: call update_test;
  li gp, 6; li x1, 0xffffffff; li x2, 0xfffffffe; bltu x1, x2, 1f; bne x0, gp, 2f; 1: bne x0, gp, fail; 2: bltu x1, x2, 1b; 3: call update_test;
  li gp, 7; li x1, 0xffffffff; li x2, 0x00000000; bltu x1, x2, 1f; bne x0, gp, 2f; 1: bne x0, gp, fail; 2: bltu x1, x2, 1b; 3: call update_test;
  li gp, 8; li x1, 0x80000000; li x2, 0x7fffffff; bltu x1, x2, 1f; bne x0, gp, 2f; 1: bne x0, gp, fail; 2: bltu x1, x2, 1b; 3: call update_test;

jr x31

bgeu_test:  li x10,0; sw x10, 0(x12)	#10 tests

  #-------------------------------------------------------------
  # Branch tests
  #-------------------------------------------------------------

  # Each test checks both forward and backward branches

  li gp, 2; li x1, 0x00000000; li x2, 0x00000000; bgeu x1, x2, 2f; bne x0, gp, fail; 1: bne x0, gp, 3f; 2: bgeu x1, x2, 1b; bne x0, gp, fail; 3: call update_test;
  li gp, 3; li x1, 0x00000001; li x2, 0x00000001; bgeu x1, x2, 2f; bne x0, gp, fail; 1: bne x0, gp, 3f; 2: bgeu x1, x2, 1b; bne x0, gp, fail; 3: call update_test;
  li gp, 4; li x1, 0xffffffff; li x2, 0xffffffff; bgeu x1, x2, 2f; bne x0, gp, fail; 1: bne x0, gp, 3f; 2: bgeu x1, x2, 1b; bne x0, gp, fail; 3: call update_test;
  li gp, 5; li x1, 0x00000001; li x2, 0x00000000; bgeu x1, x2, 2f; bne x0, gp, fail; 1: bne x0, gp, 3f; 2: bgeu x1, x2, 1b; bne x0, gp, fail; 3: call update_test;
  li gp, 6; li x1, 0xffffffff; li x2, 0xfffffffe; bgeu x1, x2, 2f; bne x0, gp, fail; 1: bne x0, gp, 3f; 2: bgeu x1, x2, 1b; bne x0, gp, fail; 3: call update_test;
  li gp, 7; li x1, 0xffffffff; li x2, 0x00000000; bgeu x1, x2, 2f; bne x0, gp, fail; 1: bne x0, gp, 3f; 2: bgeu x1, x2, 1b; bne x0, gp, fail; 3: call update_test;

  li gp, 8; li x1, 0x00000000; li x2, 0x00000001; bgeu x1, x2, 1f; bne x0, gp, 2f; 1: bne x0, gp, fail; 2: bgeu x1, x2, 1b; 3: call update_test;
  li gp, 9; li x1, 0xfffffffe; li x2, 0xffffffff; bgeu x1, x2, 1f; bne x0, gp, 2f; 1: bne x0, gp, fail; 2: bgeu x1, x2, 1b; 3: call update_test;
  li gp, 10; li x1, 0x00000000; li x2, 0xffffffff; bgeu x1, x2, 1f; bne x0, gp, 2f; 1: bne x0, gp, fail; 2: bgeu x1, x2, 1b; 3: call update_test;
  li gp, 11; li x1, 0x7fffffff; li x2, 0x80000000; bgeu x1, x2, 1f; bne x0, gp, 2f; 1: bne x0, gp, fail; 2: bgeu x1, x2, 1b; 3: call update_test;
jr x31

auipc_test:  li x10,0; sw x10, 0(x12)		#2 tests
#.align 3
  .align 3; lla x28, 1f + 10000; jal x27, 1f; 1: sub x28, x28, x27;; li x29, ((10000) & ((1 << (32 - 1) << 1) - 1)); li gp, 2; bne x28, x29, fail; call update_test;
  .align 3; lla x28, 1f - 10000; jal x27, 1f; 1: sub x28, x28, x27;; li x29, ((-10000) & ((1 << (32 - 1) << 1) - 1)); li gp, 3; bne x28, x29, fail; call update_test;
jr x31

lui_test:  li x10,0; sw x10, 0(x12)		#5 tests

  #-------------------------------------------------------------
  # Basic tests
  #-------------------------------------------------------------

  lui x1, 0x00000; li x29, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 2; bne x1, x29, fail; call update_test;
  lui x1, 0xfffff;sra x1,x1,1; li x29, ((0xfffffffffffff800) & ((1 << (32 - 1) << 1) - 1)); li gp, 3; bne x1, x29, fail; call update_test;
  lui x1, 0x7ffff;sra x1,x1,20; li x29, ((0x00000000000007ff) & ((1 << (32 - 1) << 1) - 1)); li gp, 4; bne x1, x29, fail; call update_test;
  lui x1, 0x80000;sra x1,x1,20; li x29, ((0xfffffffffffff800) & ((1 << (32 - 1) << 1) - 1)); li gp, 5; bne x1, x29, fail; call update_test;

  lui x0, 0x80000; li x29, ((0) & ((1 << (32 - 1) << 1) - 1)); li gp, 6; bne x0, x29, fail; call update_test;
  
jr x31

jal_test:  li x10,0; sw x10, 0(x12)		#2 test

  #-------------------------------------------------------------
  # Test 2: Basic test
  #-------------------------------------------------------------


  li gp, 2
  li ra, 0

  jal x4, target_2
linkaddr_2:
  nop
  nop

  j fail

target_2:
  la x2, linkaddr_2
  bne x2, x4, fail
  call update_test;

  li ra, 1; jal x0, 1f; addi ra, ra, 1; addi ra, ra, 1; addi ra, ra, 1; addi ra, ra, 1; 1: addi ra, ra, 1; addi ra, ra, 1;; li x29, ((3) & ((1 << (32 - 1) << 1) - 1)); li gp, 3; bne ra, x29, fail; call update_test;

jr x31

jalr_test:  li x10,0; sw x10, 0(x12)		#4 test

  #-------------------------------------------------------------
  # Test 2: Basic test
  #-------------------------------------------------------------

test_2a:
  li gp, 2
  li t0, 0
  la t1, target_2a

  jalr t0, t1, 0
linkaddr_2a:
  j fail

target_2a:
  la t1, linkaddr_2a
  bne t0, t1, fail
  call update_test;

  li gp, 4; li x4, 0; 1: la x6, 2f; jalr x19, x6, 0; bne x0, gp, fail; 2: addi x4, x4, 1; li x5, 2; bne x4, x5, 1b; call update_test;
  li gp, 5; li x4, 0; 1: la x6, 2f; nop; jalr x19, x6, 0; bne x0, gp, fail; 2: addi x4, x4, 1; li x5, 2; bne x4, x5, 1b;   call update_test;
  li gp, 6; li x4, 0; 1: la x6, 2f; nop; nop; jalr x19, x6, 0; bne x0, gp, fail; 2: addi x4, x4, 1; li x5, 2; bne x4, x5, 1b;   call update_test;

jr x31



lh_test:  li x10,0; sw x10, 0(x12)		#10 test

  #-------------------------------------------------------------
  # Basic tests
  #-------------------------------------------------------------

   la x1, tdatlh; lh x30, 0(x1);; li x29, ((0x00000000000000ff) & ((1 << (32 - 1) << 1) - 1)); li gp, 2; bne x30, x29, fail; call update_test;
   la x1, tdatlh; lh x30, 2(x1);; li x29, ((0xffffffffffffff00) & ((1 << (32 - 1) << 1) - 1)); li gp, 3; bne x30, x29, fail; call update_test;
   la x1, tdatlh; lh x30, 4(x1);; li x29, ((0x0000000000000ff0) & ((1 << (32 - 1) << 1) - 1)); li gp, 4; bne x30, x29, fail; call update_test;
   la x1, tdatlh; lh x30, 6(x1);; li x29, ((0xfffffffffffff00f) & ((1 << (32 - 1) << 1) - 1)); li gp, 5; bne x30, x29, fail; call update_test;

  # Test with negative offset

   la x1, tdat4lh; lh x30, -6(x1);; li x29, ((0x00000000000000ff) & ((1 << (32 - 1) << 1) - 1)); li gp, 6; bne x30, x29, fail; call update_test;
   la x1, tdat4lh; lh x30, -4(x1);; li x29, ((0xffffffffffffff00) & ((1 << (32 - 1) << 1) - 1)); li gp, 7; bne x30, x29, fail; call update_test;
   la x1, tdat4lh; lh x30, -2(x1);; li x29, ((0x0000000000000ff0) & ((1 << (32 - 1) << 1) - 1)); li gp, 8; bne x30, x29, fail; call update_test;
   la x1, tdat4lh; lh x30, 0(x1);; li x29, ((0xfffffffffffff00f) & ((1 << (32 - 1) << 1) - 1)); li gp, 9; bne x30, x29, fail; call update_test;

  # Test with a negative base

   la x1, tdatlh; addi x1, x1, -32; lh x5, 32(x1);; li x29, ((0x00000000000000ff) & ((1 << (32 - 1) << 1) - 1)); li gp, 10; bne x5, x29, fail; call update_test;

  # Test with unaligned base

   la x1, tdatlh; addi x1, x1, -5; lh x5, 7(x1);; li x29, ((0xffffffffffffff00) & ((1 << (32 - 1) << 1) - 1)); li gp, 11; bne x5, x29, fail; call update_test;
jr x31

lb_test:  li x10,0; sw x10, 0(x12)		#10 test
	  slli x10,x10,1;

  #-------------------------------------------------------------
  # Basic tests
  #-------------------------------------------------------------

   la x1, tdatlb; lb x30, 0(x1);; li x29, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); li gp, 2; bne x30, x29, fail; call update_test;
   la x1, tdatlb; lb x30, 1(x1);; li x29, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 3; bne x30, x29, fail; call update_test;
   la x1, tdatlb; lb x30, 2(x1);; li x29, ((0xfffffffffffffff0) & ((1 << (32 - 1) << 1) - 1)); li gp, 4; bne x30, x29, fail; call update_test;
   la x1, tdatlb; lb x30, 3(x1);; li x29, ((0x000000000000000f) & ((1 << (32 - 1) << 1) - 1)); li gp, 5; bne x30, x29, fail; call update_test;

  # Test with negative offset

   la x1, tdat4lb; lb x30, -3(x1);; li x29, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); li gp, 6; bne x30, x29, fail; call update_test;
   la x1, tdat4lb; lb x30, -2(x1);; li x29, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 7; bne x30, x29, fail; call update_test;
   la x1, tdat4lb; lb x30, -1(x1);; li x29, ((0xfffffffffffffff0) & ((1 << (32 - 1) << 1) - 1)); li gp, 8; bne x30, x29, fail; call update_test;
   la x1, tdat4lb; lb x30, 0(x1);; li x29, ((0x000000000000000f) & ((1 << (32 - 1) << 1) - 1)); li gp, 9; bne x30, x29, fail; call update_test;

  # Test with a negative base

   la x1, tdatlb; addi x1, x1, -32; lb x5, 32(x1);; li x29, ((0xffffffffffffffff) & ((1 << (32 - 1) << 1) - 1)); li gp, 10; bne x5, x29, fail; call update_test;

  # Test with unaligned base

   la x1, tdatlb; addi x1, x1, -6; lb x5, 7(x1);; li x29, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 11; bne x5, x29, fail; call update_test;

jr x31


lbu_test:  li x10,0; sw x10, 0(x12)		#10 test
	  slli x10,x10,1;

  #-------------------------------------------------------------
  # Basic tests
  #-------------------------------------------------------------

   la x1, tdatlb; lbu x30, 0(x1);; li x29, ((0x00000000000000ff) & ((1 << (32 - 1) << 1) - 1)); li gp, 2; bne x30, x29, fail; call update_test;
   la x1, tdatlb; lbu x30, 1(x1);; li x29, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 3; bne x30, x29, fail; call update_test;
   la x1, tdatlb; lbu x30, 2(x1);; li x29, ((0x00000000000000f0) & ((1 << (32 - 1) << 1) - 1)); li gp, 4; bne x30, x29, fail; call update_test;
   la x1, tdatlb; lbu x30, 3(x1);; li x29, ((0x000000000000000f) & ((1 << (32 - 1) << 1) - 1)); li gp, 5; bne x30, x29, fail; call update_test;

  # Test with negative offset

   la x1, tdat4lb; lbu x30, -3(x1);; li x29, ((0x00000000000000ff) & ((1 << (32 - 1) << 1) - 1)); li gp, 6; bne x30, x29, fail; call update_test;
   la x1, tdat4lb; lbu x30, -2(x1);; li x29, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 7; bne x30, x29, fail; call update_test;
   la x1, tdat4lb; lbu x30, -1(x1);; li x29, ((0x00000000000000f0) & ((1 << (32 - 1) << 1) - 1)); li gp, 8; bne x30, x29, fail; call update_test;
   la x1, tdat4lb; lbu x30, 0(x1);; li x29, ((0x000000000000000f) & ((1 << (32 - 1) << 1) - 1)); li gp, 9; bne x30, x29, fail; call update_test;

  # Test with a negative base

   la x1, tdatlb; addi x1, x1, -32; lbu x5, 32(x1);; li x29, ((0x00000000000000ff) & ((1 << (32 - 1) << 1) - 1)); li gp, 10; bne x5, x29, fail; call update_test;


  # Test with unaligned base

   la x1, tdatlb; addi x1, x1, -6; lbu x5, 7(x1);; li x29, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 11; bne x5, x29, fail; call update_test;

jr x31


lhu_test:  li x10,0; sw x10, 0(x12)		#10 test
	  slli x10,x10,1;

  #-------------------------------------------------------------
  # Basic tests
  #-------------------------------------------------------------

   la x1, tdatlh; lhu x30, 0(x1);; li x29, ((0x00000000000000ff) & ((1 << (32 - 1) << 1) - 1)); li gp, 2; bne x30, x29, fail; call update_test;
   la x1, tdatlh; lhu x30, 2(x1);; li x29, ((0x000000000000ff00) & ((1 << (32 - 1) << 1) - 1)); li gp, 3; bne x30, x29, fail; call update_test;
   la x1, tdatlh; lhu x30, 4(x1);; li x29, ((0x0000000000000ff0) & ((1 << (32 - 1) << 1) - 1)); li gp, 4; bne x30, x29, fail; call update_test;
   la x1, tdatlh; lhu x30, 6(x1);; li x29, ((0x000000000000f00f) & ((1 << (32 - 1) << 1) - 1)); li gp, 5; bne x30, x29, fail; call update_test;

  # Test with negative offset

   la x1, tdat4lh; lhu x30, -6(x1);; li x29, ((0x00000000000000ff) & ((1 << (32 - 1) << 1) - 1)); li gp, 6; bne x30, x29, fail; call update_test;
   la x1, tdat4lh; lhu x30, -4(x1);; li x29, ((0x000000000000ff00) & ((1 << (32 - 1) << 1) - 1)); li gp, 7; bne x30, x29, fail; call update_test;
   la x1, tdat4lh; lhu x30, -2(x1);; li x29, ((0x0000000000000ff0) & ((1 << (32 - 1) << 1) - 1)); li gp, 8; bne x30, x29, fail; call update_test;
   la x1, tdat4lh; lhu x30, 0(x1);; li x29, ((0x000000000000f00f) & ((1 << (32 - 1) << 1) - 1)); li gp, 9; bne x30, x29, fail; call update_test;

  # Test with a negative base

   la x1, tdatlh; addi x1, x1, -32; lhu x5, 32(x1);; li x29, ((0x00000000000000ff) & ((1 << (32 - 1) << 1) - 1)); li gp, 10; bne x5, x29, fail; call update_test;

  # Test with unaligned base

   la x1, tdatlh; addi x1, x1, -5; lhu x5, 7(x1);; li x29, ((0x000000000000ff00) & ((1 << (32 - 1) << 1) - 1)); li gp, 11; bne x5, x29, fail; call update_test;

jr x31

sh_test:  li x10,0; sw x10, 0(x12)		#10 test
	  slli x10,x10,1;

  #-------------------------------------------------------------
  # Basic tests
  #-------------------------------------------------------------

   la x1, tdatsh; li x2, 0x00000000000000aa; sh x2, 0(x1); lh x30, 0(x1);; li x29, ((0x00000000000000aa) & ((1 << (32 - 1) << 1) - 1)); li gp, 2; bne x30, x29, fail; call update_test;
   la x1, tdatsh; li x2, 0xffffffffffffaa00; sh x2, 2(x1); lh x30, 2(x1);; li x29, ((0xffffffffffffaa00) & ((1 << (32 - 1) << 1) - 1)); li gp, 3; bne x30, x29, fail; call update_test;
   la x1, tdatsh; li x2, 0xffffffffbeef0aa0; sh x2, 4(x1); lh x30, 4(x1);; li x29, ((0xffffffff00000aa0) & ((1 << (32 - 1) << 1) - 1)); li gp, 4; bne x30, x29, fail; call update_test;
   la x1, tdatsh; li x2, 0xffffffffffffa00a; sh x2, 6(x1); lh x30, 6(x1);; li x29, ((0xffffffffffffa00a) & ((1 << (32 - 1) << 1) - 1)); li gp, 5; bne x30, x29, fail; call update_test;

  # Test with negative offset

   la x1, tdat8sh; li x2, 0x00000000000000aa; sh x2, -6(x1); lh x30, -6(x1);; li x29, ((0x00000000000000aa) & ((1 << (32 - 1) << 1) - 1)); li gp, 6; bne x30, x29, fail; call update_test;
   la x1, tdat8sh; li x2, 0xffffffffffffaa00; sh x2, -4(x1); lh x30, -4(x1);; li x29, ((0xffffffffffffaa00) & ((1 << (32 - 1) << 1) - 1)); li gp, 7; bne x30, x29, fail; call update_test;
   la x1, tdat8sh; li x2, 0x0000000000000aa0; sh x2, -2(x1); lh x30, -2(x1);; li x29, ((0x0000000000000aa0) & ((1 << (32 - 1) << 1) - 1)); li gp, 8; bne x30, x29, fail; call update_test;
   la x1, tdat8sh; li x2, 0xffffffffffffa00a; sh x2, 0(x1); lh x30, 0(x1);; li x29, ((0xffffffffffffa00a) & ((1 << (32 - 1) << 1) - 1)); li gp, 9; bne x30, x29, fail; call update_test;

  # Test with a negative base

   la x1, tdat9sh; li x2, 0x12345678; addi x4, x1, -32; sh x2, 32(x4); lh x5, 0(x1);; li x29, ((0x5678) & ((1 << (32 - 1) << 1) - 1)); li gp, 10; bne x5, x29, fail; call update_test;

  # Test with unaligned base

   la x1, tdat9sh; li x2, 0x00003098; addi x1, x1, -5; sh x2, 7(x1); la x4, tdat10sh; lh x5, 0(x4);; li x29, ((0x3098) & ((1 << (32 - 1) << 1) - 1)); li gp, 11; bne x5, x29, fail; call update_test;
jr x31

sb_test:  li x10,0; sw x10, 0(x12)		#10 test
	  slli x10,x10,1;

  #-------------------------------------------------------------
  # Basic tests
  #-------------------------------------------------------------

   la x1, tdatsb; li x2, 0xffffffffffffffaa; sb x2, 0(x1); lb x30, 0(x1);; li x29, ((0xffffffffffffffaa) & ((1 << (32 - 1) << 1) - 1)); li gp, 2; bne x30, x29, fail; call update_test;
   la x1, tdatsb; li x2, 0x0000000000000000; sb x2, 1(x1); lb x30, 1(x1);; li x29, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 3; bne x30, x29, fail; call update_test;
   la x1, tdatsb; li x2, 0xffffffffffffefa0; sb x2, 2(x1); lb x30, 2(x1);; li x29, ((0xffffffffffffffa0) & ((1 << (32 - 1) << 1) - 1)); li gp, 4; bne x30, x29, fail; call update_test;
   la x1, tdatsb; li x2, 0x000000000000000a; sb x2, 3(x1); lb x30, 3(x1);; li x29, ((0x000000000000000a) & ((1 << (32 - 1) << 1) - 1)); li gp, 5; bne x30, x29, fail; call update_test;

  # Test with negative offset

   la x1, tdat8sb; li x2, 0xffffffffffffffaa; sb x2, -3(x1); lb x30, -3(x1);; li x29, ((0xffffffffffffffaa) & ((1 << (32 - 1) << 1) - 1)); li gp, 6; bne x30, x29, fail; call update_test;
   la x1, tdat8sb; li x2, 0x0000000000000000; sb x2, -2(x1); lb x30, -2(x1);; li x29, ((0x0000000000000000) & ((1 << (32 - 1) << 1) - 1)); li gp, 7; bne x30, x29, fail; call update_test;
   la x1, tdat8sb; li x2, 0xffffffffffffffa0; sb x2, -1(x1); lb x30, -1(x1);; li x29, ((0xffffffffffffffa0) & ((1 << (32 - 1) << 1) - 1)); li gp, 8; bne x30, x29, fail; call update_test;
   la x1, tdat8sb; li x2, 0x000000000000000a; sb x2, 0(x1); lb x30, 0(x1);; li x29, ((0x000000000000000a) & ((1 << (32 - 1) << 1) - 1)); li gp, 9; bne x30, x29, fail; call update_test;

  # Test with a negative base

   la x1, tdat9sb; li x2, 0x12345678; addi x4, x1, -32; sb x2, 32(x4); lb x5, 0(x1);; li x29, ((0x78) & ((1 << (32 - 1) << 1) - 1)); li gp, 10; bne x5, x29, fail; call update_test;

  # Test with unaligned base

   la x1, tdat9sb; li x2, 0x00003098; addi x1, x1, -6; sb x2, 7(x1); la x4, tdat10sb; lb x5, 0(x4);; li x29, ((0xffffffffffffff98) & ((1 << (32 - 1) << 1) - 1)); li gp, 11; bne x5, x29, fail; call update_test;

jr x31


.data

tdatlw:
tdat1lw: .word 0x00ff00ff
tdat2lw: .word 0xff00ff00
tdat3lw: .word 0x0ff00ff0
tdat4lw: .word 0xf00ff00f

tdat:
tdat1: .word 0xdeadbeef
tdat2: .word 0xdeadbeef
tdat3: .word 0xdeadbeef
tdat4: .word 0xdeadbeef
tdat5: .word 0xdeadbeef
tdat6: .word 0xdeadbeef
tdat7: .word 0xdeadbeef
tdat8: .word 0xdeadbeef
tdat9: .word 0xdeadbeef

tdatlh:
tdat1lh: .half 0x00ff
tdat2lh: .half 0xff00
tdat3lh: .half 0x0ff0
tdat4lh: .half 0xf00f

tdatlb:
tdat1lb: .byte 0xff
tdat2lb: .byte 0x00
tdat3lb: .byte 0xf0
tdat4lb: .byte 0x0f

tdatsb:
tdat1sb: .byte 0xef
tdat2sb: .byte 0xef
tdat3sb: .byte 0xef
tdat4sb: .byte 0xef
tdat5sb: .byte 0xef
tdat6sb: .byte 0xef
tdat7sb: .byte 0xef
tdat8sb: .byte 0xef
tdat9sb: .byte 0xef
tdat10sb: .byte 0xef

tdatsh:
tdat1sh: .half 0xbeef
tdat2sh: .half 0xbeef
tdat3sh: .half 0xbeef
tdat4sh: .half 0xbeef
tdat5sh: .half 0xbeef
tdat6sh: .half 0xbeef
tdat7sh: .half 0xbeef
tdat8sh: .half 0xbeef
tdat9sh: .half 0xbeef
tdat10sh: .half 0xbeef

.align 4; .global end_signature; end_signature:
