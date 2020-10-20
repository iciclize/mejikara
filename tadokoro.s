	.file	"tadokoro.c"
__SP_L__ = 0x3d
__SREG__ = 0x3f
__tmp_reg__ = 0
__zero_reg__ = 1
	.text
.global	fifo_write
	.type	fifo_write, @function
fifo_write:
/* prologue: function */
/* frame size = 0 */
/* stack size = 0 */
.L__stack_usage = 0
	lds r18,fifoWp
	lds r25,fifoRp
	cp r18,r25
	breq .L1
	lds r30,fifoWp
	mov __tmp_reg__,r30
	lsl r0
	sbc r31,r31
	subi r30,lo8(-(fifoBuf))
	sbci r31,hi8(-(fifoBuf))
	st Z,r24
	lds r24,fifoWp
	subi r24,lo8(-(1))
	andi r24,lo8(7)
	sts fifoWp,r24
.L1:
	ret
	.size	fifo_write, .-fifo_write
.global	fifo_read
	.type	fifo_read, @function
fifo_read:
/* prologue: function */
/* frame size = 0 */
/* stack size = 0 */
.L__stack_usage = 0
	lds r24,fifoRp
	mov __tmp_reg__,r24
	lsl r0
	sbc r25,r25
	adiw r24,1
	andi r24,7
	clr r25
	lds r18,fifoWp
	mov __tmp_reg__,r18
	lsl r0
	sbc r19,r19
	cp r24,r18
	cpc r25,r19
	breq .L7
	sts fifoRp,r24
.L7:
	lds r30,fifoRp
	mov __tmp_reg__,r30
	lsl r0
	sbc r31,r31
	subi r30,lo8(-(fifoBuf))
	sbci r31,hi8(-(fifoBuf))
	ld r24,Z
	ret
	.size	fifo_read, .-fifo_read
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
	lds r25,div_count
	lds r24,timer_div
	cp r25,r24
	brsh .L9
	lds r24,div_count
	subi r24,lo8(-(1))
	sts div_count,r24
	rjmp .L8
.L9:
	rcall fifo_read
	out 0x2b,r24
	sts div_count,__zero_reg__
.L8:
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
.global	i2c_start
	.type	i2c_start, @function
i2c_start:
/* prologue: function */
/* frame size = 0 */
/* stack size = 0 */
.L__stack_usage = 0
	sbi 0x18,0
	ldi r24,lo8(6)
1:	dec r24
	brne 1b
	rjmp .
	sbi 0x18,1
	ldi r24,lo8(6)
1:	dec r24
	brne 1b
	rjmp .
	cbi 0x18,0
	ldi r24,lo8(6)
1:	dec r24
	brne 1b
	rjmp .
	cbi 0x18,1
	ret
	.size	i2c_start, .-i2c_start
.global	i2c_stop
	.type	i2c_stop, @function
i2c_stop:
/* prologue: function */
/* frame size = 0 */
/* stack size = 0 */
.L__stack_usage = 0
	cbi 0x18,0
	ldi r24,lo8(6)
1:	dec r24
	brne 1b
	rjmp .
	sbi 0x18,1
	ldi r24,lo8(6)
1:	dec r24
	brne 1b
	rjmp .
	sbi 0x18,0
	ret
	.size	i2c_stop, .-i2c_stop
.global	i2c_reset
	.type	i2c_reset, @function
i2c_reset:
/* prologue: function */
/* frame size = 0 */
/* stack size = 0 */
.L__stack_usage = 0
	sbi 0x18,0
	ldi r24,lo8(6)
1:	dec r24
	brne 1b
	rjmp .
	ldi r24,lo8(8)
.L14:
	sbi 0x18,1
	ldi r25,lo8(6)
1:	dec r25
	brne 1b
	rjmp .
	cbi 0x18,1
	ldi r25,lo8(6)
1:	dec r25
	brne 1b
	rjmp .
	subi r24,lo8(-(-1))
	brne .L14
	sbi 0x18,1
	ldi r24,lo8(6)
1:	dec r24
	brne 1b
	rjmp .
	cbi 0x18,0
	ldi r25,lo8(6)
1:	dec r25
	brne 1b
	rjmp .
	sbi 0x18,1
	ldi r24,lo8(6)
1:	dec r24
	brne 1b
	rjmp .
	sbi 0x18,0
	ldi r25,lo8(6)
1:	dec r25
	brne 1b
	rjmp .
	cbi 0x18,1
	ldi r24,lo8(6)
1:	dec r24
	brne 1b
	rjmp .
	cbi 0x18,0
	ldi r25,lo8(6)
1:	dec r25
	brne 1b
	rjmp .
	ret
	.size	i2c_reset, .-i2c_reset
.global	i2c_transmit
	.type	i2c_transmit, @function
i2c_transmit:
/* prologue: function */
/* frame size = 0 */
/* stack size = 0 */
.L__stack_usage = 0
	ldi r18,lo8(8)
	ldi r19,0
	ldi r25,lo8(-128)
.L19:
	mov r20,r24
	and r20,r25
	breq .L17
	sbi 0x18,0
	rjmp .L18
.L17:
	cbi 0x18,0
.L18:
	ldi r20,lo8(6)
1:	dec r20
	brne 1b
	rjmp .
	sbi 0x18,1
	ldi r20,lo8(6)
1:	dec r20
	brne 1b
	rjmp .
	cbi 0x18,1
	ldi r20,lo8(6)
1:	dec r20
	brne 1b
	rjmp .
	lsr r25
	subi r18,1
	sbc r19,__zero_reg__
	brne .L19
	sbi 0x18,0
	cbi 0x17,0
	ldi r24,lo8(6)
1:	dec r24
	brne 1b
	rjmp .
	sbi 0x18,1
	in r24,0x16
	ldi r25,lo8(6)
1:	dec r25
	brne 1b
	rjmp .
	cbi 0x18,1
	sbi 0x17,0
	andi r24,lo8(1)
	ret
	.size	i2c_transmit, .-i2c_transmit
.global	i2c_receive
	.type	i2c_receive, @function
i2c_receive:
/* prologue: function */
/* frame size = 0 */
/* stack size = 0 */
.L__stack_usage = 0
	mov r18,r24
	sbi 0x18,0
	cbi 0x17,0
	ldi r25,lo8(8)
	ldi r24,0
.L23:
	lsl r24
	sbi 0x18,1
	ldi r19,lo8(6)
1:	dec r19
	brne 1b
	rjmp .
	sbic 0x16,0
	subi r24,lo8(-(1))
.L22:
	cbi 0x18,1
	ldi r19,lo8(6)
1:	dec r19
	brne 1b
	rjmp .
	subi r25,lo8(-(-1))
	brne .L23
	sbi 0x17,0
	tst r18
	breq .L24
	sbi 0x18,0
	rjmp .L25
.L24:
	cbi 0x18,0
.L25:
	ldi r25,lo8(6)
1:	dec r25
	brne 1b
	rjmp .
	sbi 0x18,1
	ldi r19,lo8(6)
1:	dec r19
	brne 1b
	rjmp .
	cbi 0x18,1
	ldi r25,lo8(6)
1:	dec r25
	brne 1b
	rjmp .
	ret
	.size	i2c_receive, .-i2c_receive
.global	ima_decode
	.type	ima_decode, @function
ima_decode:
	push r12
	push r13
	push r14
	push r15
	push r28
	push r29
/* prologue: function */
/* frame size = 0 */
/* stack size = 6 */
.L__stack_usage = 6
	movw r26,r22
	mov r25,r24
	andi r25,lo8(15)
	ld r18,X
	ldi r19,0
	movw r30,r18
	lsl r30
	rol r31
	subi r30,lo8(-(ima_step_table))
	sbci r31,hi8(-(ima_step_table))
/* #APP */
 ;  210 "tadokoro.c" 1
	lpm r28, Z+
	lpm r29, Z
	
 ;  0 "" 2
/* #NOAPP */
	movw r20,r28
	ldi r22,3
	1:
	lsr r21
	ror r20
	dec r22
	brne 1b
	ldi r22,0
	ldi r23,0
	sbrs r24,2
	rjmp .L31
	add r20,r28
	adc r21,r29
	adc r22,__zero_reg__
	adc r23,__zero_reg__
.L31:
	sbrs r24,1
	rjmp .L32
	movw r30,r28
	lsr r31
	ror r30
	add r20,r30
	adc r21,r31
	adc r22,__zero_reg__
	adc r23,__zero_reg__
.L32:
	sbrs r24,0
	rjmp .L33
	lsr r29
	ror r28
	lsr r29
	ror r28
	add r20,r28
	adc r21,r29
	adc r22,__zero_reg__
	adc r23,__zero_reg__
.L33:
	sbrs r24,3
	rjmp .L34
	com r23
	com r22
	com r21
	neg r20
	sbci r21,lo8(-1)
	sbci r22,lo8(-1)
	sbci r23,lo8(-1)
.L34:
	adiw r26,1
	ld r12,X+
	ld r13,X
	sbiw r26,1+1
	mov __tmp_reg__,r13
	lsl r0
	sbc r14,r14
	sbc r15,r15
	add r20,r12
	adc r21,r13
	adc r22,r14
	adc r23,r15
	cp r20,__zero_reg__
	ldi r24,-128
	cpc r21,r24
	ldi r24,-1
	cpc r22,r24
	cpc r23,r24
	brge .L35
	ldi r20,0
	ldi r21,lo8(-128)
	rjmp .L53
.L35:
	cp r20,__zero_reg__
	ldi r24,-128
	cpc r21,r24
	cpc r22,__zero_reg__
	cpc r23,__zero_reg__
	brlt .L53
	ldi r20,lo8(-1)
	ldi r21,lo8(127)
.L53:
	adiw r26,1+1
	st X,r21
	st -X,r20
	sbiw r26,1
	mov r30,r25
	ldi r31,0
	subi r30,lo8(-(ima_index_table))
	sbci r31,hi8(-(ima_index_table))
/* #APP */
 ;  223 "tadokoro.c" 1
	lpm r30, Z
	
 ;  0 "" 2
/* #NOAPP */
	add r18,r30
	adc r19,__zero_reg__
	sbrc r30,7
	dec r19
	sbrc r19,7
	rjmp .L38
	st X,r18
	cpi r18,89
	cpc r19,__zero_reg__
	brlt .L40
	ldi r24,lo8(88)
	st X,r24
	rjmp .L40
.L38:
	st X,__zero_reg__
.L40:
	adiw r26,1
	ld r24,X+
	ld r25,X
	sbiw r26,1+1
/* epilogue start */
	pop r29
	pop r28
	pop r15
	pop r14
	pop r13
	pop r12
	ret
	.size	ima_decode, .-ima_decode
.global	play
	.type	play, @function
play:
	push r2
	push r3
	push r4
	push r5
	push r6
	push r7
	push r8
	push r9
	push r10
	push r11
	push r12
	push r13
	push r14
	push r15
	push r16
	push r17
	push r28
	push r29
	in r28,__SP_L__
	clr r29
	subi r28,lo8(-(-59))
	out __SP_L__,r28
/* prologue: function */
/* frame size = 59 */
/* stack size = 77 */
.L__stack_usage = 77
	std Y+57,__zero_reg__
	mov r2,__zero_reg__
	mov r3,__zero_reg__
	ldi r16,0
	ldi r17,0
	movw r4,r28
	ldi r18,53
	add r4,r18
	adc r5,__zero_reg__
	movw r24,r28
	adiw r24,55
	std Y+56,r25
	std Y+55,r24
.L55:
	sts play_irq,__zero_reg__
	ldi r25,lo8(27)
	out 0x17,r25
	in r24,0x18
	andi r24,lo8(-28)
	out 0x18,r24
	in r24,0x3b
	ori r24,lo8(64)
	out 0x3b,r24
	ldi r30,lo8(1)
	sts fifoWp,r30
	sts fifoRp,__zero_reg__
	lds r30,sound_index
	ldi r31,0
	lsl r30
	rol r31
	subi r30,lo8(-(sound_table))
	sbci r31,hi8(-(sound_table))
/* #APP */
 ;  268 "tadokoro.c" 1
	lpm r12, Z+
	lpm r13, Z
	
 ;  0 "" 2
/* #NOAPP */
	ldi r31,12
	add r12,r31
	adc r13,__zero_reg__
	rcall i2c_reset
	rcall i2c_start
	ldi r24,lo8(-96)
	rcall i2c_transmit
	mov r24,r13
	rcall i2c_transmit
	mov r24,r12
	rcall i2c_transmit
	rcall i2c_start
	ldi r24,lo8(-95)
	rcall i2c_transmit
	movw r24,r28
	adiw r24,1
	movw r10,r24
	movw r14,r24
.L56:
	ldi r24,0
	rcall i2c_receive
	movw r30,r10
	st Z+,r24
	movw r10,r30
	cp r4,r30
	cpc r5,r31
	brne .L56
	ldi r24,lo8(1)
	rcall i2c_receive
	rcall i2c_stop
	ldi r24,0
	ldi r25,0
.L80:
	movw r30,r14
	add r30,r24
	adc r31,r25
	ldd r20,Z+4
	ldd r21,Z+5
	ldd r22,Z+6
	ldd r23,Z+7
	movw r10,r24
	ldi r31,8
	add r10,r31
	adc r11,__zero_reg__
	movw r30,r14
	add r30,r24
	adc r31,r25
	ld r6,Z
	ldd r7,Z+1
	ldd r8,Z+2
	ldd r9,Z+3
	ldi r18,100
	cp r6,r18
	ldi r18,97
	cpc r7,r18
	ldi r18,116
	cpc r8,r18
	ldi r18,97
	cpc r9,r18
	brne .+2
	rjmp .L58
	ldi r30,102
	cp r6,r30
	ldi r30,97
	cpc r7,r30
	ldi r30,99
	cpc r8,r30
	ldi r30,116
	cpc r9,r30
	breq .L59
	ldi r31,102
	cp r6,r31
	ldi r31,109
	cpc r7,r31
	ldi r31,116
	cpc r8,r31
	ldi r31,32
	cpc r9,r31
	breq .+2
	rjmp .L54
	movw r30,r14
	add r30,r10
	adc r31,r11
	ld r18,Z
	ldd r19,Z+1
	cpi r18,1
	cpc r19,__zero_reg__
	breq .L82
	cpi r18,17
	cpc r19,__zero_reg__
	breq .+2
	rjmp .L54
	movw r30,r14
	add r30,r24
	adc r31,r25
	ldd r2,Z+20
	ldd r3,Z+21
	ldi r18,lo8(1)
	std Y+57,r18
	rjmp .L61
.L82:
	std Y+57,__zero_reg__
.L61:
	movw r30,r14
	add r30,r24
	adc r31,r25
	ldd r18,Z+10
	ldd r19,Z+11
	cpi r18,1
	cpc r19,__zero_reg__
	breq .+2
	rjmp .L54
	movw r30,r14
	add r30,r24
	adc r31,r25
	ldd r24,Z+12
	ldd r25,Z+13
	std Y+59,r25
	std Y+58,r24
	rjmp .L91
.L59:
	movw r30,r14
	add r30,r10
	adc r31,r11
	ld r16,Z
	ldd r17,Z+1
.L91:
	movw r24,r10
	add r24,r20
	adc r25,r21
	rjmp .L80
.L58:
	cp r16,__zero_reg__
	cpc r17,__zero_reg__
	brne .L64
	movw r16,r20
.L64:
	ldi r31,lo8(6)
	out 0x27,r31
	ldi r18,lo8(-1)
	out 0x2d,r18
	ldi r20,lo8(80)
	out 0x2c,r20
	ldi r24,lo8(1)
	out 0x30,r24
	out 0x2f,__zero_reg__
	ldd r18,Y+58
	ldd r19,Y+59
	subi r18,100
	sbc r19,__zero_reg__
	ldi r20,0
	ldi r21,0
	ldi r22,lo8(-128)
	ldi r23,lo8(-124)
	ldi r24,lo8(30)
	ldi r25,0
	rcall __udivmodsi4
	lds r24,timer_div
.L81:
	ldi r25,lo8(1)
	add r25,r24
	cpi r18,1
	ldi r30,1
	cpc r19,r30
	brlo .L92
	lsr r19
	ror r18
	mov r24,r25
	rjmp .L81
.L92:
	sts timer_div,r24
	subi r18,lo8(-(-1))
	out 0x29,r18
	ldi r24,lo8(2)
	out 0x2a,r24
	out 0x33,r24
	ldi r24,lo8(16)
	out 0x39,r24
	rcall i2c_start
	ldi r24,lo8(-96)
	rcall i2c_transmit
	movw r24,r12
	add r24,r10
	adc r25,r11
	mov r24,r25
	rcall i2c_transmit
	mov r24,r12
	add r24,r10
	rcall i2c_transmit
	rcall i2c_start
	ldi r24,lo8(-95)
	rcall i2c_transmit
	ldd r31,Y+57
	cpi r31,lo8(1)
	breq .+2
	rjmp .L68
	movw r6,r2
	ldi r18,4
	sub r6,r18
	ldi r18,-128
	sbc r7,r18
	lsl r6
	rol r7
	movw r8,r6
	ldi r20,-1
	sub r8,r20
	sbc r9,r20
	com r6
	com r7
.L75:
	ldi r24,0
	rcall i2c_receive
	mov r13,r24
	ldi r24,0
	rcall i2c_receive
	mov r18,r13
	ldi r19,0
	or r19,r24
	std Y+3,r19
	std Y+2,r18
	ldi r24,0
	rcall i2c_receive
	std Y+1,r24
	ldi r24,0
	rcall i2c_receive
/* #APP */
 ;  400 "tadokoro.c" 1
	cli
 ;  0 "" 2
/* #NOAPP */
	ldd r24,Y+3
	subi r24,lo8(-(-128))
	rcall fifo_write
/* #APP */
 ;  402 "tadokoro.c" 1
	sei
 ;  0 "" 2
/* #NOAPP */
	movw r12,r8
.L72:
	ldi r24,0
	rcall i2c_receive
	std Y+53,r24
	swap r24
	andi r24,lo8(15)
	std Y+54,r24
	lds r24,play_irq
	cpse r24,__zero_reg__
	rjmp .L55
	movw r10,r4
.L71:
/* #APP */
 ;  416 "tadokoro.c" 1
	cli
 ;  0 "" 2
/* #NOAPP */
	movw r22,r14
	movw r30,r10
	ld r24,Z+
	movw r10,r30
	rcall ima_decode
	ldi r24,lo8(-128)
	add r24,r25
	rcall fifo_write
/* #APP */
 ;  419 "tadokoro.c" 1
	sei
 ;  0 "" 2
/* #NOAPP */
.L70:
	lds r25,fifoRp
	lds r24,fifoWp
	cp r25,r24
	breq .L70
	ldd r24,Y+55
	ldd r25,Y+56
	cp r10,r24
	cpc r11,r25
	brne .L71
	ldi r25,2
	sub r12,r25
	sbc r13,__zero_reg__
	ldi r30,2
	cp r12,r30
	cpc r13,__zero_reg__
	brsh .L72
	add r16,r6
	adc r17,r7
	cpi r16,4
	cpc r17,__zero_reg__
	brlo .+2
	rjmp .L75
.L79:
	ldi r24,lo8(1)
	rcall i2c_receive
	rcall i2c_stop
	rjmp .L76
.L77:
	lds r25,fifoRp
	lds r24,fifoWp
	cp r25,r24
	breq .L77
	subi r16,1
	sbc r17,__zero_reg__
.L68:
	cp r16,__zero_reg__
	cpc r17,__zero_reg__
	breq .L79
	ldi r24,0
	rcall i2c_receive
/* #APP */
 ;  433 "tadokoro.c" 1
	cli
 ;  0 "" 2
/* #NOAPP */
	rcall fifo_write
/* #APP */
 ;  435 "tadokoro.c" 1
	sei
 ;  0 "" 2
/* #NOAPP */
	lds r24,play_irq
	tst r24
	breq .L77
	std Y+57,__zero_reg__
	rjmp .L55
.L76:
	lds r24,fifoRp
	lds r18,fifoWp
	mov __tmp_reg__,r24
	lsl r0
	sbc r25,r25
	adiw r24,1
	andi r24,7
	clr r25
	mov __tmp_reg__,r18
	lsl r0
	sbc r19,r19
	cp r24,r18
	cpc r25,r19
	brne .L76
	ldi r24,lo8(64)
	out 0x2c,r24
	out 0x30,__zero_reg__
	in r24,0x18
	ori r24,lo8(27)
	out 0x18,r24
.L54:
/* epilogue start */
	subi r28,lo8(-(59))
	out __SP_L__,r28
	pop r29
	pop r28
	pop r17
	pop r16
	pop r15
	pop r14
	pop r13
	pop r12
	pop r11
	pop r10
	pop r9
	pop r8
	pop r7
	pop r6
	pop r5
	pop r4
	pop r3
	pop r2
	ret
	.size	play, .-play
.global	__vector_1
	.type	__vector_1, @function
__vector_1:
	push r1
	push r0
	in r0,__SREG__
	push r0
	clr __zero_reg__
	push r24
/* prologue: Signal */
/* frame size = 0 */
/* stack size = 4 */
.L__stack_usage = 4
	ldi r24,lo8(1)
	sts play_irq,r24
/* epilogue start */
	pop r24
	pop r0
	out __SREG__,r0
	pop r0
	pop r1
	reti
	.size	__vector_1, .-__vector_1
	.section	.text.startup,"ax",@progbits
.global	main
	.type	main, @function
main:
/* prologue: function */
/* frame size = 0 */
/* stack size = 0 */
.L__stack_usage = 0
	sbi 0x18,2
	out 0x35,__zero_reg__
	ldi r24,lo8(64)
	out 0x3b,r24
	ldi r28,lo8(9)
.L95:
	lds r24,sound_index
	mov r22,r28
	rcall __udivmodqi4
	sts sound_index,r25
	rcall play
	in r24,0x35
	andi r24,lo8(-25)
	ori r24,lo8(16)
	out 0x35,r24
	in r24,0x35
	ori r24,lo8(32)
	out 0x35,r24
/* #APP */
 ;  470 "tadokoro.c" 1
	sleep
	
 ;  0 "" 2
/* #NOAPP */
	in r24,0x35
	andi r24,lo8(-33)
	out 0x35,r24
	lds r24,sound_index
	subi r24,lo8(-(1))
	sts sound_index,r24
	rjmp .L95
	.size	main, .-main
.global	ima_step_table
	.section	.progmem.data,"a",@progbits
	.type	ima_step_table, @object
	.size	ima_step_table, 178
ima_step_table:
	.word	7
	.word	8
	.word	9
	.word	10
	.word	11
	.word	12
	.word	13
	.word	14
	.word	16
	.word	17
	.word	19
	.word	21
	.word	23
	.word	25
	.word	28
	.word	31
	.word	34
	.word	37
	.word	41
	.word	45
	.word	50
	.word	55
	.word	60
	.word	66
	.word	73
	.word	80
	.word	88
	.word	97
	.word	107
	.word	118
	.word	130
	.word	143
	.word	157
	.word	173
	.word	190
	.word	209
	.word	230
	.word	253
	.word	279
	.word	307
	.word	337
	.word	371
	.word	408
	.word	449
	.word	494
	.word	544
	.word	598
	.word	658
	.word	724
	.word	796
	.word	876
	.word	963
	.word	1060
	.word	1166
	.word	1282
	.word	1411
	.word	1552
	.word	1707
	.word	1878
	.word	2066
	.word	2272
	.word	2499
	.word	2749
	.word	3024
	.word	3327
	.word	3660
	.word	4026
	.word	4428
	.word	4871
	.word	5358
	.word	5894
	.word	6484
	.word	7132
	.word	7845
	.word	8630
	.word	9493
	.word	10442
	.word	11487
	.word	12635
	.word	13899
	.word	15289
	.word	16818
	.word	18500
	.word	20350
	.word	22385
	.word	24623
	.word	27086
	.word	29794
	.word	32767
.global	ima_index_table
	.type	ima_index_table, @object
	.size	ima_index_table, 16
ima_index_table:
	.byte	-1
	.byte	-1
	.byte	-1
	.byte	-1
	.byte	2
	.byte	4
	.byte	6
	.byte	8
	.byte	-1
	.byte	-1
	.byte	-1
	.byte	-1
	.byte	2
	.byte	4
	.byte	6
	.byte	8
	.comm	fifoBuf,8,1
	.comm	fifoRp,1,1
	.comm	fifoWp,1,1
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
.global	play_irq
	.type	play_irq, @object
	.size	play_irq, 1
play_irq:
	.zero	1
.global	sound_index
	.type	sound_index, @object
	.size	sound_index, 1
sound_index:
	.zero	1
.global	sound_table
	.section	.progmem.data,"a",@progbits
	.type	sound_table, @object
	.size	sound_table, 18
sound_table:
	.word	0
	.word	5180
	.word	10490
	.word	16312
	.word	28534
	.word	-32204
	.word	-28942
	.word	-25680
	.word	-21906
.global	__fuse
	.section	.fuse,"aw",@progbits
	.type	__fuse, @object
	.size	__fuse, 3
__fuse:
	.byte	-31
	.byte	-33
	.byte	-1
	.ident	"GCC: (GNU) 5.4.0"
.global __do_clear_bss
