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
.EQU bit3 = 0b00001000
.EQU output = 0b11111111
.EQU stack_init = 0x3FFF	
.EQU prescalar = 1024
.EQU sysclk = 2000000
.EQU reciprocal = 1/.01 ;idk how to spell reciprocal
.EQU offset =0 ;correcting for imprecision
;*******************************END OF EQUATES*******************************

;*********************************DEFS********************************
.def global_r20 = r21
;*******************************END OF DEFS*******************************

;***********MAIN PROGRAM*******************************
.CSEG
.org 0x0000
	rjmp main

.CSEG
.org TCC0_OVF_vect
	rjmp TC_INT
.ORG PORTF_INT0_vect
	rjmp S2_INT

.CSEG
.ORG 0x0200
main:
	;set stack pointer
	ldi r16, low(stack_init)
	out CPU_SPL, r16
	ldi r16, high(stack_init)
	out CPU_SPH, r16
	;initialization subroutines
	rcall port_init
	rcall s2_int_init
	ldi r16, 0b00100000
	loop:
		sts PORTD_OUTTGL,r16
	rjmp loop

end:
	rjmp end

;***********END MAIN PROGRAM*******************************
;****************************************************
; Name:	PORT_INIT
; Purpose: TO INITIALIZE INPUT AND OUTPUT PORTS
; Input(s): S2_SLB (PORTF_PIN3)
; Output: SLB_LEDS (PORTC)
;****************************************************
PORT_INIT:
	;save registers
	push r16
	;set s2 slb as input
	ldi r16, bit3
	sts PORTF_DIRCLR,r16
	;set slb_leds as outputs
	ldi r16,output
	sts PORTC_DIRSET,r16
	;invert SLB LEDS by using a mask
	ldi r16,0xff
	sts PORTCFG_MPCMASK,r16
	ldi r16,0b01000000
	sts PORTC_PIN0CTRL,r16
	;set green led as output
	ldi r16,0b00100000
	sts PORTD_DIRSET,r16
	ldi r16, 0b11011111
	sts PORTD_OUT,r16
	;restore from stack
	pop r16
RET
;****************************************************
; Name:	S2_INT_INIT
; Purpose: TO INITIALIZE S2 INTERRUPTS
; Input(s): N/A
; Output: N/A
;****************************************************
S2_INT_INIT:
	;save registers
	push r16
	;set s2 on slb as interrupt source
		;sets interrupt level as medium
	ldi r16, 0b00000010
	sts PORTF_INTCTRL,r16
		;sets s2 slb as interrupt source
	ldi r16, bit3
	sts PORTF_INT0MASK,r16
	;set interrupt trigger to rising edge
	ldi r16, 0b00000001
	sts PORTF_PIN3CTRL,r16
	;set PMIC
	ldi r16, 0b00000010
	sts PMIC_CTRL, r16
	;enable global interrupts
	sei
	;restore from stack
	pop r16
RET
;****************************************************
; Name:	S2_INT
; Purpose: THE S2 ISR
; Input(s): N/A
; Output: SLB_LEDS (PORTC)
;****************************************************
S2_INT:
	;save registers from stack
	push r16
	lds r16, CPU_SREG
	push r16
	;disable io interrupts
	ldi r16, 0b00000000
	sts PORTF_INTCTRL,r16
	rcall init_tc_int
	rcall init_tc
	pop r16
	sts CPU_SREG, r16
	pop r16
RETI

;****************************************************
; Name: INIT_TC
; Purpose: To initialize the relevant timer/counter modules, as pertains to
;		   application.
; Input(s): N/A
; Output: N/A
;****************************************************
INIT_TC:
push r16
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
pop r16

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
;enable tcc0 ovf interrupts, set priority to medium
ldi r16, 0b00000010
sts TCC0_INTCTRLA,r16
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
;disable TC
ldi r16, 0b00000000
sts TCC0_CTRLA, r16
;disable tc interrupt
ldi r16, 0b00000000
sts TCC0_INTCTRLA,r16
;reset period and counter
sts TCC0_PER, r16
sts TCC0_PER+1, r16
sts TCC0_CNT,r16
sts TCC0_CNT+1,r16

;get s2 slb switch status
lds r16, PORTF_IN

;increment if on
sbrc r16,3
rjmp enable
;increment global count register
inc global_r20
;display count on leds
sts PORTC_OUT,global_r20

;enable io interrupt
enable:
ldi r16, 0b00000010
sts PORTF_INTCTRL,r16

;pop registers and sreg from stack
pop r16
pop r20
sts CPU_SREG, r20
pop r20
reti
