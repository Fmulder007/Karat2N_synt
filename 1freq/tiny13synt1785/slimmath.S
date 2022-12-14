
; simple math routine for multiplication and division

;***************************************************
;* Mutiply 32x32 -> 64 bit
;;  

.global tmultiply
.global tdivide

; https://sites.google.com/site/avrasmintro/home/2b-basic-math
;;  Arg 1 in R25-R24-r23-r22
;;  Arg 2 in R21-R20-r19-r18
;;  return in R25-r24-r23-r22-r21-r20-r19-r18 

tmultiply:
        push    r2              ; save working registers
        push    r3              ; 
        push    r4
        push    r5
		push    r6
		push    r7
        push    r16
        ;clr     r16       ; clear  register
		
        ;CLR   r0         ;Initialize Answer to zero 
        ;CLR   r1         ;
        ;CLR   r2         ;
        ;CLR   r3         ; 
        CLR   r4         ;
        CLR   r5         ;
        CLR   r6         ;
        SUB   r7,r7    ;Clear r7 and Carry Flag
        MOV   r0,r22      ;Copy Multiplier to Answer
        MOV   r1,r23      ;
        MOV   r2,r24      ;
        MOV   r3,r25      ; 
        LDI   r16,33         ;Set Loop Counter to 33 
LOOP: 
        ROR   r3         ;Shift Multiplier to right
        ROR   r2         ;
        ROR   r1         ;
        ROR   r0         ;
        DEC   r16            ;Decrement Loop Counter 
        BREQ DONE         ;Check if all bits processed 
        BRCC SKIP        ;If Carry Clear skip addition
        ADD   r4,r18      ;Add Multipicand into Answer
        ADC   r5,r19      ;
        ADC   r6,r20      ;
        ADC   r7,r21      ;
SKIP:
        ROR   r7         ;Shift high bytes of Answer
        ROR   r6         ;
        ROR   r5         ;
        ROR   r4         ;
        RJMP LOOP
DONE:
		MOV   r25,r7      ;
        MOV   r24,r6      ;
        MOV   r23,r5      ;
        MOV   r22,r4      ; 
		MOV   r21,r3      ;
        MOV   r20,r2      ;
        MOV   r19,r1      ;
        MOV   r18,r0      ; 
		;; restore registers
        pop     r16
		pop     r7
		pop     r6
        pop     r5
        pop     r4
        pop     r3
        pop     r2
        clr     r1              ; clear zero register
		RET
;************************ END of 32 x 32 bit Multiplication ************************
;************************************************************
;         Start of 32 % 32 bit division             *
;   http://www.avrfreaks.net/comment/4819#comment-4819    *
;************************************************************
 ; Dividend will hold the integer result
tdivide: 
	push    r2              ; save working registers
	push    r3              ; 
	push    r16
	clr     r3       ; clear  register
	clr     r2       ; clear  register
	clr     r1       ; clear  register
	clr     r0       ; clear  register	
    ldi    r16,33                      ;1   init loop counter 
tdivide_label1: 
    sec                                    ;1   set carry 
tdivide_label2: 
    ROL    r22                       ;1   shift left dividend 
    ROL    r23                       ;1 
    ROL    r24                       ;1 
    ROL    r25                       ;1 
    ;rol    dividend5
    ;rol    dividend6
    dec    r16                         ;1 
    brne   tdivide_label3                ;1/2 
	;shape up structures
	MOV   r21,r3      ;
	MOV   r20,r2      ;
	MOV   r19,r1      ;
	MOV   r18,r0      ; 

	;; restore registers
	pop     r16
	pop     r3
	pop     r2
	clr     r1              ; clear zero register	
    ret                                    ;4 

tdivide_label3: 
    rol    r0                      ;1 
    rol    r1                      ;1 
    rol    r2                      ;1 
    rol    r3                      ;1
    ;rol    remainder5
    ;rol    remainder6
    sub    r0,r18             ;1   rem -= divisor 
    sbc    r1,r19             ;1 
    sbc    r2,r20             ;1 
    sbc    r3,r21             ;1 
    ;sbc    remainder5,divisor5
    ;sbc    remainder6,divisor6
    brcc   tdivide_label1                ;1/2 if res < 0 
    add    r0,r18             ;1   restore remainder 
    adc    r1,r19             ;1 
    adc    r2,r20             ;1 
    adc    r3,r21             ;1 
    ;adc    remainder5,divisor5
    ;adc    remainder6,divisor6 
    clc                                    ;1   clear carry 
    rjmp   tdivide_label2                ;2 
	
.end
