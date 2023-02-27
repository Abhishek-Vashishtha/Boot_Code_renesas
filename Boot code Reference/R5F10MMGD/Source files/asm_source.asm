;*****************************************************************************************************************





;******************************************************************************************************************

	.PUBLIC _CopyBits
	.PUBLIC _clear_ram
	.PUBLIC _meterology
	.PUBLIC _check_MET_s32IpDC_acc
	.PUBLIC _check_MET_s32InDC_acc
	.PUBLIC _INVERSEIpDCcon
	.PUBLIC _INVERSEInDCcon
	.PUBLIC _ClearRaw
	.PUBLIC _ClearRawNew
	.PUBLIC _RotateI1
	.PUBLIC _RotateI2
	.EXTERN _MET_a8IpRawCnt
	.EXTERN _MET_a8VRawCnt
	.EXTERN _MET_a8InRawCnt
	.EXTERN _MET_a8IpRawCntNew
	.EXTERN _MET_a8VRawCntNew
	.EXTERN _MET_a8InRawCntNew
	.EXTERN _MET_a8PpRawCnt
	.EXTERN _MET_a8PpRawCntNew
	.EXTERN _MET_a8PnRawCnt
	.EXTERN _MET_a8PnRawCntNew
	.EXTERN _MET_s32IpDC_acc
	.EXTERN _MET_s32InDC_acc
	.EXTERN _MET_s16IpDC_content
	.EXTERN _MET_s16InDC_content
	.EXTERN _MET_s16ip_new
	.EXTERN _MET_s16in_new
	.EXTERN _MET_u16i1_previous
	.EXTERN _MET_u16i2_previous
	.EXTERN _MET_s16delay1_gain
	.EXTERN _MET_s16delay2_gain
	.EXTERN _MET_u8IpDC_acc_f
	.EXTERN _MET_u8InDC_acc_f
	.EXTERN _MET_u8pfsign
	
_CopyBits:
	;voltage
	MOVW	AX,!_MET_a8VRawCnt
	MOVW	!_MET_a8VRawCntNew,AX
	MOVW	AX,!_MET_a8VRawCnt+2
	MOVW	!_MET_a8VRawCntNew+2,AX
	MOVW	AX,!_MET_a8VRawCnt+4
	MOVW	!_MET_a8VRawCntNew+4,AX

	;Ip - Phase current
	MOVW	AX,!_MET_a8IpRawCnt
	MOVW	!_MET_a8IpRawCntNew,AX
	MOVW	AX,!_MET_a8IpRawCnt+2
	MOVW	!_MET_a8IpRawCntNew+2,AX
	MOVW	AX,!_MET_a8IpRawCnt+4
	MOVW	!_MET_a8IpRawCntNew+4,AX
	
	;In - Neutral current 
	MOVW	AX,!_MET_a8InRawCnt
	MOVW	!_MET_a8InRawCntNew,AX
	MOVW	AX,!_MET_a8InRawCnt+2
	MOVW	!_MET_a8InRawCntNew+2,AX
	MOVW	AX,!_MET_a8InRawCnt+4
	MOVW	!_MET_a8InRawCntNew+4,AX

	; ;Pp
	; MOVW	AX,!_MET_a8PpRawCnt 	
	; MOVW	!_MET_a8PpRawCntNew,AX
	; MOVW	AX,!_MET_a8PpRawCnt 	
	; MOVW	!_MET_a8PpRawCntNew,AX
	; MOVW	AX,!_MET_a8PpRawCnt 	
	; MOVW	!_MET_a8PpRawCntNew,AX

	; ;Pn
	; MOVW	AX,!_MET_a8PnRawCnt
	; MOVW	!_MET_a8PnRawCntNew,AX
	; MOVW	AX,!_MET_a8PnRawCnt+2
	; MOVW	!_MET_a8PnRawCntNew+2,AX
	; MOVW	AX,!_MET_a8PnRawCnt+4
	; MOVW	!_MET_a8PnRawCntNew+4,AX
	
	;Pp
	MOV		A,!_MET_a8PpRawCnt+5 	
	SHL		A,5
	MOV		!_MET_a8PpRawCntNew+3,A
	MOV		A,!_MET_a8PpRawCnt+4
	SHR		A,3
	OR		A,!_MET_a8PpRawCntNew+3
	MOV		!_MET_a8PpRawCntNew+3,A
	
	MOV		A,!_MET_a8PpRawCnt+4
	SHL		A,5
	MOV		!_MET_a8PpRawCntNew+2,A
	MOV		A,!_MET_a8PpRawCnt+3
	SHR		A,3
	OR		A,!_MET_a8PpRawCntNew+2
	MOV		!_MET_a8PpRawCntNew+2,A
	
	MOV		A,!_MET_a8PpRawCnt+3
	SHL		A,5
	MOV		!_MET_a8PpRawCntNew+1,A
	MOV		A,!_MET_a8PpRawCnt+2
	SHR		A,3
	OR		A,!_MET_a8PpRawCntNew+1
	MOV		!_MET_a8PpRawCntNew+1,A

	MOV		A,!_MET_a8PpRawCnt+2
	SHL		A,5
	MOV		!_MET_a8PpRawCntNew+0,A
	MOV		A,!_MET_a8PpRawCnt+1
	SHR		A,3
	OR		A,!_MET_a8PpRawCntNew+0
	MOV		!_MET_a8PpRawCntNew+0,A

	;Pn
	MOV		A,!_MET_a8PnRawCnt+5 	
	SHL		A,5
	MOV		!_MET_a8PnRawCntNew+3,A
	MOV		A,!_MET_a8PnRawCnt+4
	SHR		A,3
	OR		A,!_MET_a8PnRawCntNew+3
	MOV		!_MET_a8PnRawCntNew+3,A
	
	MOV		A,!_MET_a8PnRawCnt+4
	SHL		A,5
	MOV		!_MET_a8PnRawCntNew+2,A
	MOV		A,!_MET_a8PnRawCnt+3
	SHR		A,3
	OR		A,!_MET_a8PnRawCntNew+2
	MOV		!_MET_a8PnRawCntNew+2,A
	
	MOV		A,!_MET_a8PnRawCnt+3
	SHL		A,5
	MOV		!_MET_a8PnRawCntNew+1,A
	MOV		A,!_MET_a8PnRawCnt+2
	SHR		A,3
	OR		A,!_MET_a8PnRawCntNew+1
	MOV		!_MET_a8PnRawCntNew+1,A

	MOV		A,!_MET_a8PnRawCnt+2
	SHL		A,5
	MOV		!_MET_a8PnRawCntNew+0,A
	MOV		A,!_MET_a8PnRawCnt+1
	SHR		A,3
	OR		A,!_MET_a8PnRawCntNew+0
	MOV		!_MET_a8PnRawCntNew+0,A
_ClearRaw:	
	CLRW            AX
	MOVW		!_MET_a8IpRawCnt,AX
	MOVW		!_MET_a8IpRawCnt+2,AX
	MOVW		!_MET_a8IpRawCnt+4,AX
	MOVW		!_MET_a8VRawCnt,AX
	MOVW		!_MET_a8VRawCnt+2,AX
	MOVW		!_MET_a8VRawCnt+4,AX
	MOVW		!_MET_a8InRawCnt,AX
	MOVW		!_MET_a8InRawCnt+2,AX
	MOVW		!_MET_a8InRawCnt+4,AX
	MOVW		!_MET_a8PpRawCnt,AX
	MOVW		!_MET_a8PpRawCnt+2,AX
	MOVW		!_MET_a8PpRawCnt+4,AX
	MOVW		!_MET_a8PnRawCnt,AX
	MOVW		!_MET_a8PnRawCnt+2,AX
	MOVW		!_MET_a8PnRawCnt+4,AX
	
	RET
_ClearRawNew:
	CLRW            AX
	MOVW		!_MET_a8IpRawCntNew,AX
	MOVW		!_MET_a8IpRawCntNew+2,AX
	MOVW		!_MET_a8IpRawCntNew+4,AX
	MOVW		!_MET_a8VRawCntNew,AX
	MOVW		!_MET_a8VRawCntNew+2,AX
	MOVW		!_MET_a8VRawCntNew+4,AX
	MOVW		!_MET_a8InRawCntNew,AX
	MOVW		!_MET_a8InRawCntNew+2,AX
	MOVW		!_MET_a8InRawCntNew+4,AX
	MOVW		!_MET_a8PpRawCntNew,AX
	MOVW		!_MET_a8PpRawCntNew+2,AX
	MOVW		!_MET_a8PpRawCntNew+4,AX
	MOVW		!_MET_a8PnRawCntNew,AX
	MOVW		!_MET_a8PnRawCntNew+2,AX
	MOVW		!_MET_a8PnRawCntNew+4,AX
	RET
 
_RotateI1:
	CLR1		CY
	MOVW		AX,!_MET_a8IpRawCntNew
	ROLWC		AX,1
	MOVW		!_MET_a8IpRawCntNew,AX
	MOVW		AX,!_MET_a8IpRawCntNew+2
	ROLWC		AX,1
	MOVW		!_MET_a8IpRawCntNew+2,AX
	MOVW		AX,!_MET_a8IpRawCntNew+4
	ROLWC		AX,1
	MOVW		!_MET_a8IpRawCntNew+4,AX
	RET
_RotateI2:
	CLR1		CY
	MOVW		AX,!_MET_a8InRawCntNew
	ROLWC		AX,1
	MOVW		!_MET_a8InRawCntNew,AX
	MOVW		AX,!_MET_a8InRawCntNew+2
	ROLWC		AX,1
	MOVW		!_MET_a8InRawCntNew+2,AX
	MOVW		AX,!_MET_a8InRawCntNew+4
	ROLWC		AX,1
	MOVW		!_MET_a8InRawCntNew+4,AX
	RET

_clear_ram:
	PUSH	HL
	; clear external variables which doesn't have initial value (near)
	MOVW	HL,#LOWW(STARTOF(.bss))      ; STACK start ADDRESS 
;	MOV     A, #0x0E
;	ADD     A,L
;	MOV     L,A
	MOVW	AX,#LOWW(STARTOF(.bss) + SIZEOF(.bss)) ; STACK end ADDRESS 
	BR	$.L2_BSS
.L1_BSS:
	MOV	[HL+0],#0
	INCW	HL
.L2_BSS:
	CMPW	AX,HL
	BNZ	$.L1_BSS

	; clear saddr variables which doesn't have initial value
	; 0XFFE20 --- 0XFFF1F
	MOVW	HL,#LOWW(STARTOF(.sbss))
	MOVW	AX,#LOWW(STARTOF(.sbss) + SIZEOF(.sbss))
	BR	$.L2_SBSS
.L1_SBSS:
	MOV	[HL+0],#0
	INCW	HL
.L2_SBSS:
	CMPW	AX,HL
	BNZ	$.L1_SBSS

; clear external variables which doesn't have initial value (far)
;	MOV	ES,#HIGHW(STARTOF(.bssf))
;	MOVW	HL,#LOWW(STARTOF(.bssf))
;	MOVW	AX,#LOWW(STARTOF(.bssf) + SIZEOF(.bssf))
;	BR	$.L2_BSSF
;.L1_BSSF:
;	MOV	ES:[HL+0],#0
;	INCW	HL
;.L2_BSSF:
;	CMPW	AX,HL
;	BNZ	$.L1_BSSF
    POP	HL    
	RET
	
_meterology:
	;IpDC_acc addition
	PUSH	BC
	MOVW	AX,!DSADCR2
	XCH		A,X
	ADD 	A,!_MET_s32IpDC_acc
	MOV		!_MET_s32IpDC_acc,A
	MOV		A,X
	ADDC	A,!_MET_s32IpDC_acc+1
	MOV		!_MET_s32IpDC_acc+1,A
	MOV		A,X
	BT		A.7,$PUTff_MET_s32IpDC_acc1	
	CLRB	A
	BR		$CL_MET_s32IpDC_acc1

PUTff_MET_s32IpDC_acc1:
	MOV		A,#255
	
CL_MET_s32IpDC_acc1:	
	ADDC	A,!_MET_s32IpDC_acc+2
	MOV		!_MET_s32IpDC_acc+2,A
	MOV		A,X
	BT		A.7,$PUTff_MET_s32IpDC_acc2
	CLRB	A
	BR		$CL_MET_s32IpDC_acc2
	
PUTff_MET_s32IpDC_acc2:
	MOV		A,#255
	
CL_MET_s32IpDC_acc2:	
	ADDC		A,!_MET_s32IpDC_acc+3
	MOV		!_MET_s32IpDC_acc+3,A
	
	;InDC_acc addition
	MOVW	AX,!DSADCR0
	XCH		A,X
	ADD 	A,!_MET_s32InDC_acc
	MOV		!_MET_s32InDC_acc,A
	MOV		A,X
	ADDC	A,!_MET_s32InDC_acc+1
	MOV		!_MET_s32InDC_acc+1,A
	MOV		A,X
	BT		A.7,$PUTff_MET_s32InDC_acc1	
	CLRB	A
	BR		$CL_MET_s32InDC_acc1
	
PUTff_MET_s32InDC_acc1:
	MOV		A,#255

CL_MET_s32InDC_acc1:	
	ADDC	A,!_MET_s32InDC_acc+2
	MOV		!_MET_s32InDC_acc+2,A
	MOV		A,X
	BT		A.7,$PUTff_MET_s32InDC_acc2
	CLRB	A
	BR		$CL_MET_s32InDC_acc2
	
PUTff_MET_s32InDC_acc2:
	MOV		A,#255
	
CL_MET_s32InDC_acc2:	
	ADDC	A,!_MET_s32InDC_acc+3
	MOV		!_MET_s32InDC_acc+3,A
	
	MOV     A, !_flag15       ; This flag denotes 2 bit masking of current channel at lower current that is <40 mA
	BF      A.7,$LABEL4       ; IF BIT IS NOT SET
	MOVW    AX,!DSADCR2
; New Change to reduce current
	BT      A.7,$LABEL3       ; IF BIT IS SET
	MOV     D,A
	MOV     A,X
	AND     A,#0xFC
	MOV		X,A
	MOV     A,D
	BR      $LABEL5

LABEL3:	
	MOV     D,A
	MOV     A,X
	OR      A,#0x03
	MOV		X,A
	MOV     A,D
	BR      $LABEL5
	
;_MET_s16ip_new = DSADCR2 - _MET_s16IpDC_content;
LABEL4:
	MOVW    AX,!DSADCR2
LABEL5:	
	SUBW    AX,!_MET_s16IpDC_content
	MOVW    !_MET_s16ip_new,AX
	
	;_MET_s16in_new = DSADCR0 - _MET_s16InDC_content;
;	MOV     A, !_flag15
;	BF      A.7,$LABEL7       ; IF BIT IS NOT SET
;	MOVW    AX,!DSADCR0
; New Change to reduce current
;	BT      A.7,$LABEL6       ; IF BIT IS SET
;	MOV     D,A
;	MOV     A,X
;	AND     A,#0xFC
;	MOV		X,A
;	MOV     A,D
;	BR      $LABEL8
;LABEL6:	
;	MOV     D,A
;	MOV     A,X
;	OR      A,#0x03
;	MOV		X,A
;	MOV     A,D
;	BR      $LABEL8
;LABEL7:
	MOVW    AX,!DSADCR0
; New Change to reduce current END
;LABEL8:
	SUBW    AX,!_MET_s16InDC_content
	MOVW    !_MET_s16in_new,AX
	
	;Current Ip call VI
	;_MET_a8IpRawCnt += _MET_s16ip_new^2;
	MOV     A,!_flag0        ; this flag denotes phase ct flag 
	BT      A.0, $nf1        ; if this flag is high i.e. current in ct element & if this flag is low means current in shunt element
	MOV     A,!_flag11       ; this flag indicates to calculate pf sign at ZERO crossing
	BF      A.5,$nf1         ; 0 not to calculate 1 : cal pf sign
	CLR1    !_flag11.5             
	MOVW	AX,!_MET_s16ip_new	;!DSADCR2
	MOV     !_MET_u8pfsign,#0x00
	BF      A.7,$nf1
	MOV     !_MET_u8pfsign,#0x01
nf1:    
	MOVW	AX,!_MET_s16ip_new;!DSADCR2
	MOVW	BC,!_MET_s16ip_new;!DSADCR2
	MULH
	XCH		A,X
	ADD 	A,!_MET_a8IpRawCnt
	MOV		!_MET_a8IpRawCnt,A
	XCH		A,X
	ADDC	A,!_MET_a8IpRawCnt+1
	MOV		!_MET_a8IpRawCnt+1,A
	XCH		A,C
	ADDC	A,!_MET_a8IpRawCnt+2
	MOV		!_MET_a8IpRawCnt+2,A
	XCH		A,B
	ADDC	A,!_MET_a8IpRawCnt+3
	MOV		!_MET_a8IpRawCnt+3,A
	CLRB	A
	ADDC	A,!_MET_a8IpRawCnt+4
	MOV		!_MET_a8IpRawCnt+4,A
	CLRB	A
	ADDC	A,!_MET_a8IpRawCnt+5
	MOV		!_MET_a8IpRawCnt+5,A
	;Voltage V
	;_MET_a8VRawCnt += DSADCR1 ^2;
	MOVW	AX,!DSADCR1
	MOVW	BC,!DSADCR1
	MULH
	XCH		A,X
	ADD 	A,!_MET_a8VRawCnt
	MOV		!_MET_a8VRawCnt,A
	XCH		A,X
	ADDC	A,!_MET_a8VRawCnt+1
	MOV		!_MET_a8VRawCnt+1,A
	XCH		A,C
	ADDC	A,!_MET_a8VRawCnt+2
	MOV		!_MET_a8VRawCnt+2,A
	XCH		A,B
	ADDC	A,!_MET_a8VRawCnt+3
	MOV		!_MET_a8VRawCnt+3,A
	CLRB	A
	ADDC	A,!_MET_a8VRawCnt+4
	MOV		!_MET_a8VRawCnt+4,A
	CLRB	A
	ADDC	A,!_MET_a8VRawCnt+5
	MOV		!_MET_a8VRawCnt+5,A
	;Current In
	;_MET_a8InRawCnt += _MET_s16in_new ^2;
	MOV     A,!_flag0		; this flag denotes phase ct flag 
	BF      A.0, $nf2 		; if this flag is high i.e. current in ct element & if this flag is low means current in shunt element
	MOV     A,!_flag11      ; this flag indicates to calculate pf sign at ZERO crossing
	BF      A.5,$nf2        ; 0 not to calculate 1 : cal pf sign
	CLR1    !_flag11.5             
	MOVW	AX,!_MET_s16in_new;!DSADCR2
	MOV     !_MET_u8pfsign,#0x00
	BF      A.7,$nf2
	MOV     !_MET_u8pfsign,#0x01
nf2:	
	MOVW	AX,!_MET_s16in_new;!DSADCR0
	MOVW	BC,!_MET_s16in_new;!DSADCR0
	MULH
	XCH		A,X
	ADD 	A,!_MET_a8InRawCnt
	MOV		!_MET_a8InRawCnt,A
	XCH		A,X
	ADDC	A,!_MET_a8InRawCnt+1
	MOV		!_MET_a8InRawCnt+1,A
	XCH		A,C
	ADDC	A,!_MET_a8InRawCnt+2
	MOV		!_MET_a8InRawCnt+2,A
	XCH		A,B
	ADDC	A,!_MET_a8InRawCnt+3
	MOV		!_MET_a8InRawCnt+3,A
	CLRB	A
	ADDC	A,!_MET_a8InRawCnt+4
	MOV		!_MET_a8InRawCnt+4,A
	CLRB	A
	ADDC	A,!_MET_a8InRawCnt+5
	MOV		!_MET_a8InRawCnt+5,A
	
	MOV     A, !_flag15
	BF      A.7,$LABEL_P0       ; IF BIT IS NOT SET
	MOVW    AX,!DSADCR2
	SUBW    AX,!_MET_s16InDC_content
	BR      $LABEL_P1
	;Power active
	;_MET_a8PpRawCnt += (_MET_s16ip_new*DSADCR1)
LABEL_P0:
	MOVW	AX,!_MET_s16ip_new;!DSADCR2
LABEL_P1:
	MOVW	BC,!DSADCR1
	MULH
	XCH		A,X
	ADD 	A,!_MET_a8PpRawCnt
	MOV		!_MET_a8PpRawCnt,A
	XCH		A,X
	ADDC	A,!_MET_a8PpRawCnt+1
	MOV		!_MET_a8PpRawCnt+1,A
	XCH		A,C
	ADDC	A,!_MET_a8PpRawCnt+2
	MOV		!_MET_a8PpRawCnt+2,A
	MOV		A,B
	ADDC	A,!_MET_a8PpRawCnt+3
	MOV		!_MET_a8PpRawCnt+3,A
	MOV		A,B
	BT		A.7,$PUTff1	
	CLRB	A
	BR		$CL1
PUTff1:
	MOV		A,#255
CL1:	
	ADDC	A,!_MET_a8PpRawCnt+4
	MOV		!_MET_a8PpRawCnt+4,A
	MOV		A,B
	BT		A.7,$PUTff2
	CLRB	A
	BR		$CL2
PUTff2:
	MOV		A,#255
CL2:	
	ADDC	A,!_MET_a8PpRawCnt+5
	MOV		!_MET_a8PpRawCnt+5,A	

	;EL Power active
	;_MET_a8PnRawCnt += (_MET_s16in_new * DSADCR1)
	MOVW	AX,!_MET_s16in_new;!DSADCR0
	MOVW	BC,!DSADCR1
	MULH
	XCH		A,X
	ADD 	A,!_MET_a8PnRawCnt
	MOV		!_MET_a8PnRawCnt,A
	XCH		A,X
	ADDC	A,!_MET_a8PnRawCnt+1
	MOV		!_MET_a8PnRawCnt+1,A
	XCH		A,C
	ADDC	A,!_MET_a8PnRawCnt+2
	MOV		!_MET_a8PnRawCnt+2,A
	MOV		A,B
	ADDC	A,!_MET_a8PnRawCnt+3
	MOV		!_MET_a8PnRawCnt+3,A
	MOV		A,B
	BT		A.7,$PUTff1n
	CLRB	A
	BR		$CL1n
PUTff1n:
	MOV		A,#255
CL1n:
	ADDC	A,!_MET_a8PnRawCnt+4
	MOV		!_MET_a8PnRawCnt+4,A
	MOV		A,B
	BT		A.7,$PUTff2n
	CLRB	A
	BR		$CL2n
PUTff2n:
	MOV		A,#255
CL2n:
	ADDC	A,!_MET_a8PnRawCnt+5
	MOV		!_MET_a8PnRawCnt+5,A	
	POP		BC
	RET

_check_MET_s32IpDC_acc:
	MOV		A,!_MET_s32IpDC_acc+3
	BT		A.7,$_INVERSEIpDCacc
	CLRB	!_MET_u8IpDC_acc_f
	RET	
_INVERSEIpDCacc:
	INC		!_MET_u8IpDC_acc_f
	MOV		A,!_MET_s32IpDC_acc
	XOR		A,#0xFF
	ADD		A,#0x01
	MOV		!_MET_s32IpDC_acc,A
	MOV		A,!_MET_s32IpDC_acc+1
	XOR		A,#0xFF
	ADDC	A,#0x00
	MOV		!_MET_s32IpDC_acc+1,A
	MOV		A,!_MET_s32IpDC_acc+2
	XOR		A,#0xFF
	ADDC	A,#0x00
	MOV		!_MET_s32IpDC_acc+2,A
	MOV		A,!_MET_s32IpDC_acc+3
	XOR		A,#0xFF
	ADDC	A,#0x00
	MOV		!_MET_s32IpDC_acc+3,A
	RET
_INVERSEIpDCcon:
	;INC		!_MET_u8IpDC_acc_f
	MOV		A,!_MET_s16IpDC_content
	XOR		A,#0xFF
	ADD		A,#0x01
	MOV		!_MET_s16IpDC_content,A
	MOV		A,!_MET_s16IpDC_content+1
	XOR		A,#0xFF
	ADDC	A,#0x00
	MOV		!_MET_s16IpDC_content+1,A
	RET
_check_MET_s32InDC_acc:
	MOV		A,!_MET_s32InDC_acc+3
	BT		A.7,$_INVERSEInDCacc
	CLRB	!_MET_u8InDC_acc_f
	RET	
_INVERSEInDCacc:
	INC		!_MET_u8InDC_acc_f
	MOV		A,!_MET_s32InDC_acc
	XOR		A,#0xFF
	ADD		A,#0x01
	MOV		!_MET_s32InDC_acc,A
	MOV		A,!_MET_s32InDC_acc+1
	XOR		A,#0xFF
	ADDC	A,#0x00
	MOV		!_MET_s32InDC_acc+1,A
	MOV		A,!_MET_s32InDC_acc+2
	XOR		A,#0xFF
	ADDC	A,#0x00
	MOV		!_MET_s32InDC_acc+2,A
	MOV		A,!_MET_s32InDC_acc+3
	XOR		A,#0xFF
	ADDC	A,#0x00
	MOV		!_MET_s32InDC_acc+3,A
	RET
_INVERSEInDCcon:
	;INC		!_MET_u8InDC_acc_f
	MOV		A,!_MET_s16InDC_content
	XOR		A,#0xFF
	ADD		A,#0x01
	MOV		!_MET_s16InDC_content,A
	MOV		A,!_MET_s16InDC_content+1
	XOR		A,#0xFF
	ADDC	A,#0x00
	MOV		!_MET_s16InDC_content+1,A
	RET	
	
	
