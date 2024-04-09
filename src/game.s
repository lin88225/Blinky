.syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb
  
  .global  Main
  .global  SysTick_Handler

  @ Definitions are in definitions.s to keep blinky.s "clean"
  .include "./src/definitions.s"

  .equ    BLINK_PERIOD, 500
  
  current_LED:
  .word 1
  win_state:
  .word 0

  .section .text

Main:
  PUSH    {R4-R5,LR}
@ button
  @ Initial count of button presses to 0
  @ Count must be maintained in memory - interrupt handlers
  @   must not rely on registers to maintain values across
  @   different invocations of the handler (i.e. across
  @   different presses of the pushbutton)
  LDR   R4, =count        @ count = 0;
  MOV   R5, #0            @
  STR   R5, [R4]          @

@ Configure USER pushbutton (GPIO Port A Pin 0 on STM32F3 Discovery
  @   kit) to use the EXTI0 external interrupt signal
  @ Determined by bits 3..0 of the External Interrrupt Control
  @   Register (EXTIICR)
  @ STM32F303 Reference Manual 12.1.3 (pg. 249)
  LDR     R4, =SYSCFG_EXTIICR1
  LDR     R5, [R4]
  BIC     R5, R5, #0b1111
  STR     R5, [R4]

  @ Enable (unmask) interrupts on external interrupt EXTI0
  @ EXTI0 corresponds to bit 0 of the Interrupt Mask Register (IMR)
  @ STM32F303 Reference Manual 14.3.1 (pg. 297)
  LDR     R4, =EXTI_IMR
  LDR     R5, [R4]
  ORR     R5, R5, #1
  STR     R5, [R4]

  @ Set falling edge detection on EXTI0
  @ EXTI0 corresponds to bit 0 of the Falling Trigger Selection
  @   Register (FTSR)
  @ STM32F303 Reference Manual 14.3.4 (pg. 298)
  LDR     R4, =EXTI_FTSR
  LDR     R5, [R4]
  ORR     R5, R5, #1
  STR     R5, [R4]

  @ Enable NVIC interrupt channel (Nested Vectored Interrupt Controller)
  @ EXTI0 corresponds to NVIC channel #6
  @ Enable channels using the NVIC Interrupt Set Enable Register (ISER)
  @ Writing a 1 to a bit enables the corresponding channel
  @ Writing a 0 to a bit has no effect
  @ STM32 Cortex-M4 Programming Manual 4.3.2 (pg. 210)
  LDR     R4, =NVIC_ISER
  MOV     R5, #(1<<6)
  STR     R5, [R4]

@ LED lights
  @ Enable GPIO port E by enabling its clock
  @ STM32F303 Reference Manual 9.4.6 (pg. 148)
  LDR     R4, =RCC_AHBENR
  LDR     R5, [R4]
  ORR     R5, R5, #(0b1 << (RCC_AHBENR_GPIOEEN_BIT))
  STR     R5, [R4]

  @ We'll blink LED LD3 (the orange LED)

  @ Configure LD3 for output
  @ by setting bits 27:26 of GPIOE_MODER to 01 (GPIO Port E Mode Register)
  @ (by BIClearing then ORRing)
  @ STM32F303 Reference Manual 11.4.1 (pg. 237)
  LDR     R4, =GPIOE_MODER
  LDR     R5, [R4]                  @ Read ...
  BIC     R5, #(0b11<<(LD3_PIN*2))  @ Modify ...
  ORR     R5, #(0b01<<(LD3_PIN*2))  @ write 01 to bits 
  STR     R5, [R4]                  @ Write 

  @@ LED 4
  LDR     R4, =GPIOE_MODER
  LDR     R5, [R4]                  @ Read ...
  BIC     R5, #(0b11<<(LD4_PIN*2))  @ Modify ...
  ORR     R5, #(0b01<<(LD4_PIN*2))  @ write 01 to bits 
  STR     R5, [R4]                  @ Write 

  @@ LED 5
  LDR     R4, =GPIOE_MODER
  LDR     R5, [R4]                  @ Read ...
  BIC     R5, #(0b11<<(LD5_PIN*2))  @ Modify ...
  ORR     R5, #(0b01<<(LD5_PIN*2))  @ write 01 to bits 
  STR     R5, [R4]                  @ Write 

  @@ LED 6 
  LDR     R4, =GPIOE_MODER
  LDR     R5, [R4]                  @ Read ...
  BIC     R5, #(0b11<<(LD6_PIN*2))  @ Modify ...
  ORR     R5, #(0b01<<(LD6_PIN*2))  @ write 01 to bits 
  STR     R5, [R4]                  @ Write 

  @@ LED 7
  LDR     R4, =GPIOE_MODER
  LDR     R5, [R4]                  @ Read ...
  BIC     R5, #(0b11<<(LD7_PIN*2))  @ Modify ...
  ORR     R5, #(0b01<<(LD7_PIN*2))  @ write 01 to bits 
  STR     R5, [R4]                  @ Write 

  @@ LED 8 
  LDR     R4, =GPIOE_MODER
  LDR     R5, [R4]                  @ Read ...
  BIC     R5, #(0b11<<(LD8_PIN*2))  @ Modify ...
  ORR     R5, #(0b01<<(LD8_PIN*2))  @ write 01 to bits 
  STR     R5, [R4]                   @ Write 

  @@ LED 9
  LDR     R4, =GPIOE_MODER
  LDR     R5, [R4]                  @ Read ...
  BIC     R5, #(0b11<<(LD9_PIN*2))  @ Modify ...
  ORR     R5, #(0b01<<(LD9_PIN*2))  @ write 01 to bits 
  STR     R5, [R4]                  @ Write 

  @@ LED 10 
  LDR     R4, =GPIOE_MODER
  LDR     R5, [R4]                  @ Read ...
  BIC     R5, #(0b11<<(LD10_PIN*2))  @ Modify ...
  ORR     R5, #(0b01<<(LD10_PIN*2))  @ write 01 to bits 
  STR     R5, [R4]                  @ Write 



  @ We'll blink LED LD3 (the orange LED) every 1s
  @ Initialise the first countdown to 1000 (1000ms)
  LDR     R4, =countdown
  LDR     R5, =BLINK_PERIOD
  STR     R5, [R4]  


  @ Configure SysTick Timer to generate an interrupt every 1ms

  @ STM32 Cortex-M4 Programming Manual 4.4.3 (pg. 225)
  LDR     R4, =SCB_ICSR             @ Clear any pre-existing interrupts
  LDR     R5, =SCB_ICSR_PENDSTCLR   @
  STR     R5, [R4]                  @

  @ STM32 Cortex-M4 Programming Manual 4.5.1 (pg. 247)
  LDR   R4, =SYSTICK_CSR            @ Stop SysTick timer
  LDR   R5, =0                      @   by writing 0 to CSR
  STR   R5, [R4]                    @   CSR is the Control and Status Register
  
  @ STM32 Cortex-M4 Programming Manual 4.5.2 (pg. 248)
  LDR   R4, =SYSTICK_LOAD           @ Set SysTick LOAD for 1ms delay
  LDR   R5, =7999                   @ Assuming 8MHz clock
  STR   R5, [R4]                    @ 

  @ STM32 Cortex-M4 Programming Manual 4.5.3 (pg. 249)
  LDR   R4, =SYSTICK_VAL            @   Reset SysTick internal counter to 0
  LDR   R5, =0x1                    @     by writing any value
  STR   R5, [R4]

  @ STM32 Cortex-M4 Programming Manual 4.4.3 (pg. 225)
  LDR   R4, =SYSTICK_CSR            @   Start SysTick timer by setting CSR to 0x7
  LDR   R5, =0x7                    @     set CLKSOURCE (bit 2) to system clock (1)
  STR   R5, [R4]                    @     set TICKINT (bit 1) to 1 to enable interrupts
                                    @     set ENABLE (bit 0) to 1

  @ Nothing else to do in Main
  @ Idle loop forever (welcome to interrupts!!)
Idle_Loop:
  B     Idle_Loop
  
End_Main:
  POP   {R4-R5,PC}


@
@ SysTick interrupt handler
@
  .type  SysTick_Handler, %function
SysTick_Handler:

  PUSH  {R4, R5, LR}

  LDR   R4, =countdown              @ if (countdown != 0) {
  LDR   R5, [R4]                    @
  CMP   R5, #0                      @
  BEQ   .LelseFire                  @

  SUB   R5, R5, #1                  @   countdown = countdown - 1;
  STR   R5, [R4]                    @

  B     .LendIfDelay                @ }
.LelseFire:
  LDR     R4, =current_LED
  LDR     R5, [R4]
  ADD    R5, R5, #1                @ current_LED++
  CMP     R5, #8                   @ if (numberOfLEDS>= current_LED)
  BLE     .LupdateCurrentLED       @ {  current_LED.update();  }
  MOV    R5, #1                    @ 

.LupdateCurrentLED:                    
  CMP R5, #2                       @ switch(current_LED) {
  BEQ .LLED4                       @   case 2:
  CMP R5, #3                       @     LED2.light();
  BEQ .LLED5                       @   break
  CMP R5, #4                       @         ...
  BEQ .LLED6                       @   default:
  CMP R5, #5                       @     LED1.light();
  BEQ .LLED7                       @  }
  CMP R5, #6
  BEQ .LLED8
  CMP R5, #7
  BEQ .LLED9
  CMP R5, #8
  BEQ .LLED10

  @ STM32F303 Reference Manual 11.4.6 (pg. 239)
  LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  EOR     R5, #(0b1<<(LD3_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD3_PIN);
  STR     R5, [R4]                  @ 

  LDR     R4, =countdown            @   countdown = BLINK_PERIOD;
  LDR     R5, =BLINK_PERIOD  @
  STR     R5, [R4]                  @

.LLED4:
  @@ LED 4
  LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  EOR     R5, #(0b1<<(LD4_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD3_PIN);
  STR     R5, [R4]                  @ 

  LDR     R4, =countdown            @   countdown = BLINK_PERIOD;
  LDR     R5, =BLINK_PERIOD  @
  STR     R5, [R4]                  @

.LLED5:
  @@ LED 5
  LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  EOR     R5, #(0b1<<(LD5_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD3_PIN);
  STR     R5, [R4]                  @ 

  LDR     R4, =countdown            @   countdown = BLINK_PERIOD;
  LDR     R5, =BLINK_PERIOD  @
  STR     R5, [R4]                  @

.LLED6:
  @@ LED 6 
  LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  EOR     R5, #(0b1<<(LD6_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD3_PIN);
  STR     R5, [R4]                  @ 

  LDR     R4, =countdown            @   countdown = BLINK_PERIOD;
  LDR     R5, =BLINK_PERIOD  @
  STR     R5, [R4]                  @

.LLED7:
  @@ LED 7
  LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  EOR     R5, #(0b1<<(LD7_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD3_PIN);
  STR     R5, [R4]                  @ 

  LDR     R4, =countdown            @   countdown = BLINK_PERIOD;
  LDR     R5, =BLINK_PERIOD  @
  STR     R5, [R4]                  @

.LLED8:
  @@ LED 8
  LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  EOR     R5, #(0b1<<(LD8_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD3_PIN);
  STR     R5, [R4]                  @ 

  LDR     R4, =countdown            @   countdown = BLINK_PERIOD;
  LDR     R5, =BLINK_PERIOD  @
  STR     R5, [R4]                  @

.LLED9:
  @@ LED 9 
  LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  EOR     R5, #(0b1<<(LD9_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD3_PIN);
  STR     R5, [R4]                  @ 

  LDR     R4, =countdown            @   countdown = BLINK_PERIOD;
  LDR     R5, =BLINK_PERIOD  @
  STR     R5, [R4]                  @

.LLED10:
  @@ LED 10 
  LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  EOR     R5, #(0b1<<(LD10_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD3_PIN);
  STR     R5, [R4]                  @ 

  LDR     R4, =countdown            @   countdown = BLINK_PERIOD;
  LDR     R5, =BLINK_PERIOD  @
  STR     R5, [R4]                  @

.LendIfDelay:                       @ }

  @ STM32 Cortex-M4 Programming Manual 4.4.3 (pg. 225)
  LDR     R4, =SCB_ICSR             @ Clear (acknowledge) the interrupt
  LDR     R5, =SCB_ICSR_PENDSTCLR   @
  STR     R5, [R4]                  @

  @ Return from interrupt handler
  POP  {R4, R5, PC}

.type  EXTI0_IRQHandler, %function
EXTI0_IRQHandler:         @ buttonPress()

  PUSH  {R4,R5,LR}

  LDR R4, =current_LED
  LDR R5, [R4]
  CMP R5, #7             @ if (current_LED == 7){
  BNE .Lno_Win           @     win_state++;  }
  LDR R4, =win_state
  ADD R5, R5, #1
  STR R5, [R4]

  @@ Win event goes here@@

  .Lno_Win:
  @@ No win event goes here @@

  @ Add one to count of button presses
  LDR   R4, =count        @ count = count + 1
  LDR   R5, [R4]          @
  ADD   R5, R5, #1        @
  STR   R5, [R4]          @

  @ Tell microcontroller that we have handled the EXTI0 interrupt
  @ By writing a 1 to bit 0 of the EXTI Pending Register (PR)
  @ (Writing 0s to bits has no effect)
  @ STM32F303 Reference Manual 14.3.6 (pg. 299)
  LDR   R4, =EXTI_PR      @ Clear (acknowledge) the interrupt
  MOV   R5, #(1<<0)       @
  STR   R5, [R4]          @

  @ Return from interrupt handler
  POP  {R4,R5,PC}


  .section .data

countdown:
  .space  4

  .end
