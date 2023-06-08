;Lab 3, Section 1
;Name: Steven Miller
;Class #: 11318
;PI Name: Anthony Stross
;Description: triggers an overflow interrupt every 84ms

;***************INCLUDES*************************************
.include "ATxmega128a1udef.inc"
;***************END OF INCLUDES******************************

;*********************************EQUATES********************************
.EQU input = 0b00000000
.EQU output = 0b11111111
.EQU prescalar = 1024
.EQU sysclk = 2000000
.EQU reciprocal = 1/.084 ;idk how to spell reciprocal
.EQU offset =-11 ;correcting for imprecision
.equ stack_init = 0x3FFF	
;*******************************END OF EQUATES*******************************

;*********************************DEFS********************************

;*******************************END OF DEFS*******************************

;***********MAIN PROGRAM*******************************
.CSEG
.org 0x0000
	rjmp main

.CSEG
.org TCC0_OVF_vect
rjmp TC_INT

.CSEG
.org 0x0200
MAIN:
	;initialize stack pointer
	ldi r16, low(stack_init)
	out CPU_SPL, r16
	ldi r16, high(stack_init)
	out CPU_SPH, r16
rcall init_tc_int
rcall init_tc

;toggle output port
loop:
	;wait for interrupt
	nop
	rjmp loop

rjmp loop
end:
rjmp end

;***********END MAIN PROGRAM*******************************

;****************************************************
; Name: INIT_TC
; Purpose: To initialize the relevant timer/counter modules, as pertains to
;		   application.
; Input(s): N/A
; Output: N/A
;****************************************************
INIT_TC:
;initialize count register
ldi r16,0
sts TCC0_CNT, r16
sts TCC0_CNT+1,r16

;initialize period register
ldi r16,low(((sysclk/prescalar)/reciprocal)+offset)
sts TCC0_PER, r16
ldi r16,high(((sysclk/prescalar)/reciprocal)+offset)
sts TCC0_PER+1,r16

;initialize clksel
ldi r16, TC_CLKSEL_DIV1024_gc
sts TCC0_CTRLA,r16
ldi r16, input
sts TCC0_CTRLB,r16

ret
;****************************************************
; Name: INIT_TC_INT
; Purpose: To initialize the OVF interrupt
; Input(s): N/A
; Output: N/A
;****************************************************
INIT_TC_INT:
;store registers
push r16
;initialize port c for output
ldi r16, output
sts PORTC_DIR, r16
;enable tcc0 ovf interrupts, set priority to medium
ldi r16, 0b00000010
sts TCC0_INTCTRLA,r16
;enable global interrupts
sts PMIC_CTRL,r16
sei
pop r16
ret
;****************************************************
; Name: TC_INT 
; Purpose: The TC interrupt service routine
; Input(s): TCC0_INTFLAGS, CPU_SREG
; Output: PORTC_OUTTGL
;****************************************************
TC_INT:
;push cpu sreg to stack
push r20
lds r20,CPU_SREG
push r20
;push registers to stack
push r16
;toggle port c output
ldi r16,0b11111111
sts PORTC_OUTTGL,r16
;pop registers and sreg from stack
pop r16
pop r20
sts CPU_SREG, r20
pop r20
reti
