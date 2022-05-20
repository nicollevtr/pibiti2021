.syntax unified
	.thumb
	.cpu cortex-m4

	.section .vector,"a",%progbits
	.weak vector
	.type vector, %object
	.size vector, .-vector

vector:
	.word 0x20020000
	.word init + 1

	.section .text.init
	.weak init
	.type init, %function

init:
	ldr r6, = 0x40023800 + 0x30
	ldr r0, = 0x08
	str r0, [r6]
	ldr r6, = 0x40020C00 + 0x00
	ldr r0, = 0x01000000
	str r0, [r6]

	ldr r6, = 0x40020C00 + 0x04
	ldr r0, = 0x00000000
	str r0, [r6]

	ldr r6, = 0x40020C00 + 0x08
	ldr r0, = 0x00000000
	str r0, [r6]

	ldr r6, = 0x40020C00 + 0x0C
	ldr r0, = 0x00000000
	str r0, [r6]

	mov r2, 0x1000
	mov r3, 0x0000
	ldr r6, = 0x40020C00 + 0x14
loop:
	str r2, [r6]
	ldr r1, =80000000
delay1:
	subs r1, 1
	bne delay1
	str r3, [r6]
	ldr r1, =80000000
delay2:
	subs r1, 1
	bne delay2

	b loop
