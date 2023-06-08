;Lab 3, Section 2a
;Name: Steven Miller
;Class #: 11318
;PI Name: Anthony Stross
;Description: triggers an interrupt every time s2 on the SLB is pressed

;***************INCLUDES*************************************
.include "ATxmega128a1udef.inc"
;***************END OF INCLUDES******************************

;*********************************EQUATES********************************
.EQU input = 0b00000000
.EQU bit3 = 0b00001000
.EQU output = 0b11111111
.equ stack_init = 0x3FFF	
;*******************************END OF EQUATES*******************************

;*********************************DEFS********************************
.def global_r20 = r20
;*******************************END OF DEFS*******************************

;***********MAIN PROGRAM*******************************
.CSEG
.org 0x0000
	rjmp main
;set S2_INT as the ISR
.CSEG
.org PORTF_INT0_vect
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
	;enable interrupt level 0
	ldi r16, 0b00000010
	sts PORTF_INTCTRL,r16
	;sets s2 slb as interrupt source
	ldi r16, bit3
	sts PORTF_INT0MASK,r16
	;set interrupt trigger to either edge
	ldi r16, 0b00000000
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
	;increment global count register
	inc global_r20
	;display count on leds
	sts PORTC_OUT,global_r20
	;restore registers
	pop r16
	sts CPU_SREG, r16
	pop r16
RETI