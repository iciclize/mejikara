	.file	"mejikara.c"
__SP_H__ = 0x3e
__SP_L__ = 0x3d
__SREG__ = 0x3f
__tmp_reg__ = 0
__zero_reg__ = 1
	.text
.global	fifo_init
	.type	fifo_init, @function
fifo_init:
/* prologue: function */
/* frame size = 0 */
/* stack size = 0 */
.L__stack_usage = 0
	ldi r24,lo8(1)
	ldi r25,0
	sts fifoWp+1,r25
	sts fifoWp,r24
	sts fifoRp+1,__zero_reg__
	sts fifoRp,__zero_reg__
	ret
	.size	fifo_init, .-fifo_init
.global	fifo_write
	.type	fifo_write, @function
fifo_write:
/* prologue: function */
/* frame size = 0 */
/* stack size = 0 */
.L__stack_usage = 0
	lds r18,fifoWp
	lds r19,fifoWp+1
	lds r20,fifoRp
	lds r21,fifoRp+1
	cp r18,r20
	cpc r19,r21
	breq .L2
	movw r30,r18
	subi r30,lo8(-(fifoBuf))
	sbci r31,hi8(-(fifoBuf))
	st Z,r24
	subi r18,-1
	sbci r19,-1
	andi r18,7
	clr r19
	sts fifoWp+1,r19
	sts fifoWp,r18
.L2:
	ret
	.size	fifo_write, .-fifo_write
.global	fifo_read
	.type	fifo_read, @function
fifo_read:
/* prologue: function */
/* frame size = 0 */
/* stack size = 0 */
.L__stack_usage = 0
	lds r18,fifoRp
	lds r19,fifoRp+1
	movw r24,r18
	adiw r24,1
	andi r24,7
	clr r25
	lds r20,fifoWp
	lds r21,fifoWp+1
	cp r24,r20
	cpc r25,r21
	brne .L6
	movw r30,r18
	rjmp .L8
.L6:
	sts fifoRp+1,r25
	sts fifoRp,r24
	movw r30,r24
.L8:
	subi r30,lo8(-(fifoBuf))
	sbci r31,hi8(-(fifoBuf))
	ld r24,Z
	ret
	.size	fifo_read, .-fifo_read
.global	fifo_isFull
	.type	fifo_isFull, @function
fifo_isFull:
/* prologue: function */
/* frame size = 0 */
/* stack size = 0 */
.L__stack_usage = 0
	ldi r24,lo8(1)
	ldi r25,0
	lds r20,fifoRp
	lds r21,fifoRp+1
	lds r18,fifoWp
	lds r19,fifoWp+1
	cp r20,r18
	cpc r21,r19
	breq .L10
	ldi r24,0
	ldi r25,0
.L10:
	ret
	.size	fifo_isFull, .-fifo_isFull
.global	__vector_10
	.type	__vector_10, @function
__vector_10:
	push r1
	push r0
	in r0,__SREG__
	push r0
	clr __zero_reg__
	push r18
	push r19
	push r20
	push r21
	push r22
	push r23
	push r24
	push r25
	push r26
	push r27
	push r30
	push r31
/* prologue: Signal */
/* frame size = 0 */
/* stack size = 15 */
.L__stack_usage = 15
	rcall fifo_read
	out 0x2b,r24
/* epilogue start */
	pop r31
	pop r30
	pop r27
	pop r26
	pop r25
	pop r24
	pop r23
	pop r22
	pop r21
	pop r20
	pop r19
	pop r18
	pop r0
	out __SREG__,r0
	pop r0
	pop r1
	reti
	.size	__vector_10, .-__vector_10
.global	i2c_clock_with_0
	.type	i2c_clock_with_0, @function
i2c_clock_with_0:
/* prologue: function */
/* frame size = 0 */
/* stack size = 0 */
.L__stack_usage = 0
	cbi 0x18,2
	sbi 0x17,0
	ldi r24,lo8(4)
1:	dec r24
	brne 1b
	cbi 0x18,0
	ldi r24,lo8(2)
1:	dec r24
	brne 1b
	rjmp .
	sbi 0x18,2
	ldi r24,lo8(6)
1:	dec r24
	brne 1b
	rjmp .
	ret
	.size	i2c_clock_with_0, .-i2c_clock_with_0
.global	i2c_clock_with_1
	.type	i2c_clock_with_1, @function
i2c_clock_with_1:
/* prologue: function */
/* frame size = 0 */
/* stack size = 0 */
.L__stack_usage = 0
	cbi 0x18,2
	sbi 0x17,0
	ldi r24,lo8(4)
1:	dec r24
	brne 1b
	sbi 0x18,0
	ldi r24,lo8(2)
1:	dec r24
	brne 1b
	rjmp .
	sbi 0x18,2
	ldi r24,lo8(6)
1:	dec r24
	brne 1b
	rjmp .
	ret
	.size	i2c_clock_with_1, .-i2c_clock_with_1
.global	i2c_clock_with_read_ack
	.type	i2c_clock_with_read_ack, @function
i2c_clock_with_read_ack:
/* prologue: function */
/* frame size = 0 */
/* stack size = 0 */
.L__stack_usage = 0
	cbi 0x18,2
	cbi 0x17,0
	sbi 0x18,0
	ldi r24,lo8(6)
1:	dec r24
	brne 1b
	rjmp .
	sbi 0x18,2
	ldi r24,lo8(2)
1:	dec r24
	brne 1b
	rjmp .
	sbis 0x16,2
	sbi 0x18,0
.L15:
	sbic 0x16,0
	sbi 0x18,3
.L16:
	ldi r24,lo8(4)
1:	dec r24
	brne 1b
	ret
	.size	i2c_clock_with_read_ack, .-i2c_clock_with_read_ack
.global	i2c_clock_with_read_1bit
	.type	i2c_clock_with_read_1bit, @function
i2c_clock_with_read_1bit:
/* prologue: function */
/* frame size = 0 */
/* stack size = 0 */
.L__stack_usage = 0
	cbi 0x18,2
	cbi 0x17,0
	sbi 0x18,0
	ldi r24,lo8(6)
1:	dec r24
	brne 1b
	rjmp .
	sbi 0x18,2
	ldi r25,lo8(2)
1:	dec r25
	brne 1b
	rjmp .
	sbis 0x16,2
	sbi 0x18,3
.L21:
	in r24,0x16
	ldi r25,lo8(4)
1:	dec r25
	brne 1b
	andi r24,lo8(1)
	ret
	.size	i2c_clock_with_read_1bit, .-i2c_clock_with_read_1bit
	.section	.text.startup,"ax",@progbits
.global	main
	.type	main, @function
main:
	push r12
	push r13
	push r14
	push r15
	push r16
	push r17
	push r28
	push r29
/* prologue: function */
/* frame size = 0 */
/* stack size = 8 */
.L__stack_usage = 8
	ldi r24,lo8(29)
	out 0x17,r24
	out 0x18,__zero_reg__
	ldi r24,lo8(6)
	out 0x27,r24
	ldi r24,lo8(-1)
	out 0x2d,r24
	out 0x2b,__zero_reg__
	ldi r24,lo8(96)
	out 0x2c,r24
	ldi r24,lo8(1)
	out 0x30,r24
	out 0x2f,__zero_reg__
	lds r18,Fs
	lds r19,Fs+1
	lds r20,Fs+2
	lds r21,Fs+3
	ldi r22,lo8(-128)
	ldi r23,lo8(-124)
	ldi r24,lo8(30)
	ldi r25,0
	rcall __udivmodsi4
	lds r24,timer_div
.L23:
	ldi r25,lo8(1)
	add r25,r24
	cpi r18,1
	ldi r22,1
	cpc r19,r22
	cpc r20,__zero_reg__
	cpc r21,__zero_reg__
	brlo .L29
	lsr r21
	ror r20
	ror r19
	ror r18
	mov r24,r25
	rjmp .L23
.L29:
	sts timer_div,r24
	subi r18,lo8(-(-1))
	out 0x29,r18
	ldi r24,lo8(100)
	out 0x29,r24
	ldi r24,lo8(2)
	out 0x2a,r24
	out 0x33,r24
	ldi r24,lo8(16)
	out 0x39,r24
/* #APP */
 ;  173 "mejikara.c" 1
	cli
 ;  0 "" 2
/* #NOAPP */
	in r24,0x18
	ori r24,lo8(5)
	out 0x18,r24
	ldi r24,lo8(13)
1:	dec r24
	brne 1b
	nop
	cbi 0x18,0
	ldi r22,lo8(13)
1:	dec r22
	brne 1b
	nop
	rcall i2c_clock_with_1
	rcall i2c_clock_with_0
	rcall i2c_clock_with_1
	rcall i2c_clock_with_0
	rcall i2c_clock_with_0
	rcall i2c_clock_with_0
	rcall i2c_clock_with_0
	rcall i2c_clock_with_0
	rcall i2c_clock_with_read_ack
	rcall i2c_clock_with_0
	rcall i2c_clock_with_0
	rcall i2c_clock_with_0
	rcall i2c_clock_with_0
	rcall i2c_clock_with_0
	rcall i2c_clock_with_0
	rcall i2c_clock_with_0
	rcall i2c_clock_with_0
	rcall i2c_clock_with_read_ack
	rcall i2c_clock_with_0
	rcall i2c_clock_with_0
	rcall i2c_clock_with_1
	rcall i2c_clock_with_0
	rcall i2c_clock_with_1
	rcall i2c_clock_with_1
	rcall i2c_clock_with_0
	rcall i2c_clock_with_0
	rcall i2c_clock_with_read_ack
	cbi 0x18,2
	sbi 0x17,0
	ldi r24,lo8(4)
1:	dec r24
	brne 1b
	sbi 0x18,0
	ldi r22,lo8(2)
1:	dec r22
	brne 1b
	rjmp .
	sbi 0x18,2
	ldi r24,lo8(13)
1:	dec r24
	brne 1b
	nop
	cbi 0x18,0
	ldi r22,lo8(13)
1:	dec r22
	brne 1b
	nop
	rcall i2c_clock_with_1
	rcall i2c_clock_with_0
	rcall i2c_clock_with_1
	rcall i2c_clock_with_0
	rcall i2c_clock_with_0
	rcall i2c_clock_with_0
	rcall i2c_clock_with_0
	rcall i2c_clock_with_1
	rcall i2c_clock_with_read_ack
	ldi r16,lo8(117)
	ldi r17,lo8(-9)
.L26:
/* #APP */
 ;  240 "mejikara.c" 1
	cli
 ;  0 "" 2
/* #NOAPP */
	rcall i2c_clock_with_read_1bit
	mov r28,r24
	rcall i2c_clock_with_read_1bit
	ror r28
	clr r28
	ror r28
	swap r24
	lsl r24
	lsl r24
	andi r24,lo8(-64)
	or r28,r24
	rcall i2c_clock_with_read_1bit
	mov r12,r24
	rcall i2c_clock_with_read_1bit
	mov r13,r24
	rcall i2c_clock_with_read_1bit
	mov r14,r24
	rcall i2c_clock_with_read_1bit
	mov r15,r24
	rcall i2c_clock_with_read_1bit
	mov r29,r24
	rcall i2c_clock_with_read_1bit
	mov r18,r12
	swap r18
	lsl r18
	andi r18,lo8(-32)
	or r24,r28
	or r24,r18
	mov r25,r13
	swap r25
	andi r25,lo8(-16)
	mov r18,r25
	or r18,r24
	mov r28,r14
	lsl r28
	lsl r28
	lsl r28
	mov r25,r28
	or r25,r18
	mov r28,r15
	lsl r28
	lsl r28
	or r28,r25
	mov r24,r29
	lsl r24
	or r24,r28
	rcall fifo_write
	rcall i2c_clock_with_0
/* #APP */
 ;  255 "mejikara.c" 1
	sei
 ;  0 "" 2
/* #NOAPP */
	lds r18,fifoRp
	lds r19,fifoRp+1
	lds r24,fifoWp
	lds r25,fifoWp+1
.L25:
	cp r18,r24
	cpc r19,r25
	breq .L25
	subi r16,1
	sbc r17,__zero_reg__
	breq .+2
	rjmp .L26
	rcall i2c_clock_with_read_1bit
	rcall i2c_clock_with_read_1bit
	rcall i2c_clock_with_read_1bit
	rcall i2c_clock_with_read_1bit
	rcall i2c_clock_with_read_1bit
	rcall i2c_clock_with_read_1bit
	rcall i2c_clock_with_read_1bit
	rcall i2c_clock_with_read_1bit
	rcall i2c_clock_with_1
	cbi 0x18,2
	sbi 0x17,0
	ldi r24,lo8(4)
1:	dec r24
	brne 1b
	cbi 0x18,0
	ldi r22,lo8(2)
1:	dec r22
	brne 1b
	rjmp .
	sbi 0x18,2
	ldi r24,lo8(13)
1:	dec r24
	brne 1b
	nop
	sbi 0x18,0
	ldi r22,lo8(13)
1:	dec r22
	brne 1b
	nop
	sbi 0x18,3
	ldi r24,0
	ldi r25,0
/* epilogue start */
	pop r29
	pop r28
	pop r17
	pop r16
	pop r15
	pop r14
	pop r13
	pop r12
	ret
	.size	main, .-main
	.comm	fifoBuf,8,1
	.comm	fifoRp,2,1
	.comm	fifoWp,2,1
.global	div_count
	.section .bss
	.type	div_count, @object
	.size	div_count, 1
div_count:
	.zero	1
.global	timer_div
	.type	timer_div, @object
	.size	timer_div, 1
timer_div:
	.zero	1
.global	Fs
	.data
	.type	Fs, @object
	.size	Fs, 4
Fs:
	.byte	-80
	.byte	54
	.byte	0
	.byte	0
	.ident	"GCC: (GNU) 5.4.0"
.global __do_copy_data
.global __do_clear_bss
