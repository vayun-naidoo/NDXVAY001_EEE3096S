/*
 * assembly.s
 *
 */

 @ DO NOT EDIT
	.syntax unified
    .text
    .global ASM_Main
    .thumb_func

@ DO NOT EDIT
vectors:
	.word 0x20002000
	.word ASM_Main + 1

@ DO NOT EDIT label ASM_Main
ASM_Main:

	@ Some code is given below for you to start with
	LDR R0, RCC_BASE  		@ Enable clock for GPIOA and B by setting bit 17 and 18 in RCC_AHBENR
	LDR R1, [R0, #0x14]
	LDR R2, AHBENR_GPIOAB	@ AHBENR_GPIOAB is defined under LITERALS at the end of the code
	ORRS R1, R1, R2
	STR R1, [R0, #0x14]

	LDR R0, GPIOA_BASE		@ Enable pull-up resistors for pushbuttons
	MOVS R1, #0b01010101
	STR R1, [R0, #0x0C]
	LDR R1, GPIOB_BASE  	@ Set pins connected to LEDs to outputs
	LDR R2, MODER_OUTPUT
	STR R2, [R1, #0]
	MOVS R2, #0         	@ NOTE: R2 will be dedicated to holding the value on the LEDs



@ TODO: Add code, labels and logic for button checks and LED patterns

main_loop:

    LDR R0, GPIOA_BASE        @ Load GPIOA base address
    LDR R4, [R0, #0x10]       @ Load GPIOA_IDR to check pushbutton state

    @ Check if no buttons are pressed
    MOVS R6, #0b1111          @ Move 0b1111 into R5 (to check PA0-PA3)
    ANDS R4, R4, R6           @ Mask only PA0-PA3 pins
    CMP R4, #0b1111           @ Check if no buttons are pressed (all inputs high)
    BEQ increment_default      @ If no buttons are pressed, use default increment

    @ Check if PA0 (SW2) is pressed
    MOVS R6, #0b1111          @ Mask PA0 (bit 0)
    ANDS R4, R4, R6           @ Update R4 value using bitwise AND
    CMP R4, #0b1110           @ Check if PA0 is pressed (low state)
    BEQ increment_by_two       @ If PA0 is pressed, increment by 2

    @ Check if PA1 is pressed
    MOVS R6, #0b1111         @ Mask PA1 (bit 1)
    ANDS R4, R4, R6           @ Update R4 value using bitwise AND
    CMP R4, #0b1101          @ Check if PA1 is pressed (low state)
    BEQ call_short_delay

    @ Check if PA3 is pressed
    MOVS R6, #0b1111          @ Mask PA1 (bit 1)
    ANDS R4, R4, R6           @ Update R4 value using bitwise AND
    CMP R4, #0b1011           @ Check if PA1 is pressed (low state)
    BEQ set_led_to_aa

    @ Check if PA3 is pressed
    MOVS R6, #0b1111      @ Mask PA1 (bit 1)
    ANDS R4, R4, R6          @ Update R4 value using bitwise AND
    CMP R4, #0b0111       @ Check if PA1 is pressed (low state)
    BEQ check_freeze

    @ Check if both PA0 and PA1 are pressed
    MOVS R6, #0b1111          @ Mask PA0 and PA1 (bits 0 and 1)
    ANDS R4, R4, R6           @ Update R4 value using bitwise AND
    CMP R4, #0b1100           @ Check if both PA0 and PA1 are pressed (both low)
    BEQ increment_by_two_short @ If both PA0 and PA1 are pressed, increment by 2 with short delay


check_freeze:
    LDR R0, frozen_state          @ Load the frozen state
    LDR R1, [R0]                  @ Load the current frozen state
    CMP R1, #0                     @ Check if frozen
    BEQ mask_leds                  @ If not frozen, update LEDs
    B main_loop                    @ If frozen, skip LED update

increment_by_two:
    ADDS R2, R2, #2           @ Increment by 2 if PA0 is pressed
    BL long_delay              @ Use long delay for 0.7 seconds
    B mask_leds                @ Go to mask LEDs

call_short_delay:
    ADDS R2, R2, #1           @ Increment by 1 if PA1 is pressed
    BL short_delay             @ Use short delay (0.3 seconds)
    B mask_leds               @ Go to mask LEDs

set_led_to_aa:
    MOVS R3, #0xAA            @ Set LED pattern to 0xAA when SW2 (PA2) is pressed
    LDR R0, GPIOB_BASE        @ Load GPIOB base address (low registers only)
    STR R3, [R0, #0x14]       @ Write 0xAA to GPIOB ODR (LEDs)
    B main_loop

increment_by_two_short:
    ADDS R2, R2, #2           @ Increment by 2 if both PA0 and PA1 are pressed
    BL short_delay             @ Use short delay (0.3 seconds)
    B mask_leds                @ Go to mask LEDs

increment_default:
    ADDS R2, R2, #1           @ Default increment by 1 if neither button is pressed
    BL long_delay              @ Use long delay for 0.7 seconds
    B mask_leds                @ Go to mask LEDs


mask_leds:
    MOVS R3, #0xFF            @ Load the mask value (0xFF) into R3
    ANDS R2, R2, R3           @ Mask off any overflow (restrict to 8 bits)
    B write_leds

write_leds:
    LDR R0, GPIOB_BASE        @ Load GPIOB base address
    STR R2, [R0, #0x14]       @ Write the LED value to GPIOB ODR
    B main_loop               @ Repeat the loop

@ Long Delay function
long_delay:
    LDR R3, LONG_DELAY_CNT    @ Load delay counter value
delay_loop:
    SUBS R3, R3, #1           @ Decrement delay counter
    BNE delay_loop            @ If not zero, continue loop
    BX LR                     @ Return to main loop

@ Short Delay function (for 0.3 seconds)
short_delay:
    LDR R3, SHORT_DELAY_CNT   @ Load short delay counter value
short_delay_loop:
    SUBS R3, R3, #1           @ Decrement delay counter
    BNE short_delay_loop      @ If not zero, continue loop
    BX LR                     @ Return to main loop

@ LITERALS
.align
RCC_BASE:           .word 0x40021000
AHBENR_GPIOAB:      .word 0b1100000000000000000
GPIOA_BASE:         .word 0x48000000
GPIOB_BASE:         .word 0x48000400
MODER_OUTPUT:       .word 0x5555
LONG_DELAY_CNT:     .word 1400000    @ Adjust for 0.7 second delay
SHORT_DELAY_CNT:    .word 600000     @ Adjust for 0.3 second delay
frozen_state:       .word 0          @ 0 - not frozen, 1 - frozen
