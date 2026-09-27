/*
 * Project2_us.c
 *
 * Created: 11/03/2025 17:37:30
 * Author : Ciaran.MacNamee
 */ 


 #include <avr/io.h>
 #include <avr/interrupt.h>
 #include <avr/cpufunc.h>
 
 #define F_CPU 20000000
 #define USART3_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)
 #define TARGET_BAUD_RATE 9600
 
 #define TCB_INTERVAL 1000000
 #define TCB_TOP 50000
 #define TCB_COUNT (TCB_INTERVAL/TCB_TOP)
 #define TICK_DURATION_US 0.1
 
 #include <util/delay.h>
 #include <stdio.h>

 /* Use a struct to make the association between PORTs and bits connected to the LED array more explicit */
 struct LED_BITS
 {
	PORT_t *LED_PORT;
	uint8_t bit_mapping;
 };

 struct LED_BITS LED_Array[10] = {
 {&PORTC, PIN5_bm}, {&PORTC, PIN4_bm}, {&PORTA, PIN0_bm}, {&PORTF, PIN5_bm}, {&PORTC, PIN6_bm}, {&PORTB, PIN2_bm}, {&PORTF, PIN4_bm}, {&PORTA, PIN1_bm}, {&PORTA, PIN2_bm}, {&PORTA, PIN3_bm}
 };
 
 uint8_t qcntr = 0, sndcntr = 0;
 unsigned char queue[50];

 uint8_t newDistanceData, newTimeData, newADC0Data;
 uint16_t adc_reading;		/* ADC0 RES has 10-bits, read it into a 16-bit variable */
 uint8_t ServoFollowADC;    /* Servo position based on ADC0 RES value */
 volatile uint8_t UserSelectedValue = 1;
 volatile uint16_t NewServoPosition = 1500;
 extern volatile uint16_t high_time;  // These should be updated in your ISR
 extern volatile uint16_t low_time;
 uint16_t step_delay = 1000;
 uint16_t servo_pos = 1500;
 volatile uint16_t pulseTime = 0;
 volatile uint16_t periodTime = 0;
 
void CLOCK_init (void);
void InitialiseLED_PORT_bits(void);
void Initialise_TCA0_SS_PWM(void);
void Initialise_EVSYS (void);
void Initialise_TCB0_ICP_PW(void);
void Initialise_TCB2_ICP_PWFRQ(void);
void Set_Clear_Ports(uint8_t set);
void USART3_init(void);
void ADC0_init(void);
void sendmsg(char* s); 


 void CLOCK_init (void)
 {
	/* Disable CLK_PER Prescaler */
	ccp_write_io( (void *) &CLKCTRL.MCLKCTRLB , (0 << CLKCTRL_PEN_bp));
	/* If set from the fuses during device programming, the CPU will now run at 20MHz (default is /6) */
 }

 void InitialiseLED_PORT_bits()
 {
	PORTC.DIRSET = PIN6_bm | PIN5_bm | PIN4_bm;  /*(1<<6) | (1<<5) | (1<<4); 0x70;*/		/* PC4-UNO D1 (TXD1), PC5-UNO D0 (RXD1), PC6 - UNO D4  */
	PORTA.DIRSET = PIN3_bm | PIN2_bm | PIN1_bm | PIN0_bm; /*(1<<1) | (1<<0);   0x0f; */      /* PA1-UNO D7, PA0 - UNO D2, PA2- LED8, PA3 - LED9  */
	PORTB.DIRSET = PIN2_bm; /*0x04;*/		/* PB2 - UNO D5 */
	PORTF.DIRSET = PIN5_bm | PIN4_bm; /*(1<<5) | (1<<4);   0x30; */		/* PF5 - UNO D3, PF4 UNO D6 */
	/* Later use PIN6_bm etc */
 }

void USART3_init(void) {
	PORTB.DIRCLR = PIN5_bm;
	PORTB.DIRSET = PIN4_bm;
	USART3.BAUD = (uint16_t)USART3_BAUD_RATE(TARGET_BAUD_RATE);
	USART3.CTRLB = (USART_TXEN_bm | USART_RXEN_bm);
	PORTMUX.USARTROUTEA |= PORTMUX_USART3_ALT1_gc;
	USART3.CTRLA = USART_TXCIE_bm;
}

void Initialise_TCA0_SS_PWM()
{
	PORTA.DIRSET = 0b00000001;  // Make PORTA Bit 0 an output (may be done in InitialiseLED_PORT_bits())
	TCA0.SINGLE.CTRLB = 0b00000011;  //Set TCA0 to Single Slope PWM (CTRLB)
	TCA0.SINGLE.PER = 24999;  //Set TCA0.SINGLE.PER or PERBUF for 50Hz PWM frequency (24999)
	TCA0.SINGLE.CMP0 = 1250;  //Set TCA0.SINGLE.CMP0 for nominal -90degrees initial position – On time = 1ms
	TCA0.SINGLE.CTRLA = 0b00001001;  //Timer/Counter TCA0 Clock Source: CLK_PER divided by 16 and TCA0 enabled (CTRLA)
}

 void Initialise_EVSYS()
 {
	/* Set Port B Pin 0 as input event this is on Channel 0 */
	EVSYS.CHANNEL0 = EVSYS_GENERATOR_PORT1_PIN0_gc; /* Connect user to event channel 0  */
    EVSYS.USERTCB0 = EVSYS_CHANNEL_CHANNEL0_gc;  /* TCB0 is the Channel 0 User */
	
	/* Set Port 0 Pin 3 (PE3) as input event this is on Channel 4 */
	EVSYS.CHANNEL4 = EVSYS_GENERATOR_PORT0_PIN3_gc;  /* Connect user to event channel 4 */
    EVSYS.USERTCB2 = EVSYS_CHANNEL_CHANNEL4_gc;/* TCB2 is the Channel 4 user */
	
	/* Set TCB3 as the Generator for any other Channel */
	EVSYS.CHANNEL2 = EVSYS_GENERATOR_TCB3_CAPT_gc;  /* ADC0 is the user of the Channel selected for TCB3 Generator */
    EVSYS.USERTCB3 = EVSYS_CHANNEL_CHANNEL2_gc;  /* TCB3 starts ADC0  */
}

 void Initialise_TCB0_ICP_PW()
 {
	TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;  /* Enable TCB0 and set CLK_PER divider to 2: Timer clock = 10MHz now */
	TCB0.CTRLB = TCB_CNTMODE_PW_gc;  /* Configure TCB0 in Input Capture Pulse Width mode */
 	TCB0.INTCTRL = TCB_CAPT_bm;   /* Enable Capture or Timeout interrupt */
 	TCB0.EVCTRL = TCB_CAPTEI_bm;  /* Enable Event Input and Event Edge, Rising Edge selected */
 }
 
 /* Later add TCB1 initialisation to detect 555 oscillation stopped */
 
 void Initialise_TCB2_ICP_PWFRQ()
 {
	 TCB2.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;  /* Enable TCB2 and set CLK_PER divider to 2: Timer clock = 10MHz now */
	 TCB2.CTRLB = TCB_CNTMODE_FRQPW_gc;  /* Configure TCB0 in Input Capture Clock Frequency Measurement mode */
	 TCB2.INTCTRL = TCB_CAPT_bm;  /* Enable Capture or Timeout interrupt */
	 TCB2.EVCTRL = TCB_CAPTEI_bm;  /* Enable Event Input and Event Edge, Rising Edge selected */
 }
 
 void TCB3_init(void)
 {
	  /* enable overflow interrupt */
	 TCB3.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;  /* PER divided by 2 and Enable the TCB0 */
	 TCB3.CTRLB = TCB_CNTMODE_INT_gc;  /* Periodic Interrupt Mode */
	 TCB3.CCMP = (F_CPU / 2) / 200;  /* Set TCB3.CCMP for 5ms interrupt rate */
	 TCB3.INTCTRL = TCB_CAPT_bm;  /* Enable the interrupt */
	 
 }
 void ADC0_init(void)
 {
	 ADC0.CTRLA = 0b00000000;    /* CTRLA: 10-bit resolution selected, Free Running Mode NOT selected, ADC0 not enabled yet */
	 ADC0.CTRLB = 0b00000000;    /* CTRLB: Simple No Accumulation operation selected, this line could be omitted */
	 ADC0.CTRLC = 0b01010110;    /* CTRLC: SAMPCAP=1; REFSEL: VDD; PRESC set to DIV128 */
	 ADC0.CTRLD = 0b00100000;    /* CTRLD: INITDLY set to 16 CLK_ADC cycles */
	 ADC0.MUXPOS = 0b00000011;   /* MUXPOS: Select AIN3 (shared with PORTD3), decision based on the Shield and adapters we use */
	 
	 ADC0.EVCTRL = 0b00000001;   /* EVCTRL: STARTEI set to 1  */
	 ADC0.INTCTRL = 0b00000001;  /* INTCTRL: Enable an interrupt when conversion complete (RESRDY) */
	 ADC0.CTRLA |= 0b00000001;	 /* Enable ADC0 and leave the other CTRLA bits unchanged, note |= */
}
 
 

 int main(void)
{
    char ch;
    char str_buffer[60];

    uint8_t continuousDistance = 0;
    uint8_t continuousTime = 0;
    uint8_t continuousVolts = 0;
	volatile uint16_t low_time = 0;
	volatile uint16_t high_time = 0;

	
    ServoFollowADC = 0;

    newDistanceData = 0;
    newTimeData = 0;
    newADC0Data = 0;

    CLOCK_init();

    /* set UNO D0-D7 to all outputs, also LED8 and LED9 */
    InitialiseLED_PORT_bits();

    Set_Clear_Ports(0);        /* Initialize LEDS to all OFF */

    Initialise_TCA0_SS_PWM();
    Initialise_EVSYS();
    Initialise_TCB0_ICP_PW();
    USART3_init();
    Initialise_TCB2_ICP_PWFRQ();
    TCB3_init();
    ADC0_init();

    sei(); /* Enable Global Interrupts */

    while (1) {
        if (USART3.STATUS & USART_RXCIF_bm) {
            ch = USART3.RXDATAL;

            switch (ch) {
                case 'a':
                case 'A':
                    sprintf(str_buffer, "ADC0 RES = %d\n", adc_reading);
                    sendmsg(str_buffer);
                    break;
                case 'v':
                case 'V': {
	                uint16_t mV = ((uint32_t)adc_reading * 5000) / 1023;
	                sprintf(str_buffer, "Voltage = %u mV\n", mV);
	                sendmsg(str_buffer);
	                break;
                }
                case 't':
                case 'T':
				{
                        if (periodTime > 0) {
	                        uint16_t freq = 1000000UL / periodTime;
	                        uint8_t duty = (pulseTime * 100UL) / periodTime;

	                        sprintf(str_buffer,
	                        "555 Timer:\nPeriod = %u us\nPulse = %u us\nFrequency = %u Hz\nDuty = %u%%\n",
	                        periodTime, pulseTime, freq, duty);
	                        sendmsg(str_buffer);
	                        } else {
	                        sendmsg("555 Timer: Invalid period\n");
                        }
                }
                    break;
                case 'H':
                case 'h':
                    {
	                    float high_us = high_time * TICK_DURATION_US;  // Convert ticks to us
	                    char msg[32];
	                    snprintf(msg, sizeof(msg), "High Time: %.2f us\n", high_us);
	                    sendmsg(msg);
	                    break;
                    }
                    break;
                case 'L':
                case 'l':
                    {
	                    float low_us = low_time * TICK_DURATION_US;
	                    char msg[32];
	                    snprintf(msg, sizeof(msg), "Low Time: %.2f us\n", low_us);
	                    sendmsg(msg);
	                    break;
                    }
                    break;
                case 'd':
                case 'D':{
                    
                    uint16_t distance_cm = pulseTime / 58;
                    sprintf(str_buffer, "Distance = %u cm\n", distance_cm);
                    sendmsg(str_buffer);
                    break;
				}
                case 's':
                case 'S':
                    continuousDistance = 1;
                    sprintf(str_buffer, "Continuous Distance ON\n");
                    sendmsg(str_buffer);
                    break;
                case 'u':
                case 'U':
                    continuousDistance = 0;
                    sprintf(str_buffer, "Continuous Distance OFF\n");
                    sendmsg(str_buffer);
                    break;
                case 'c':
                case 'C':
                    continuousTime = 1;
                    sprintf(str_buffer, "Continuous Time ON\n");
                    sendmsg(str_buffer);
                    break;
                case 'e':
                case 'E':
                    continuousTime = 0;
                    sprintf(str_buffer, "Continuous Time OFF\n");
                    sendmsg(str_buffer);
                    break;
                case 'm':
                case 'M':
                    continuousVolts = 1;        
					sprintf(str_buffer, "Continuous Volts ON\n");    
                    sendmsg(str_buffer);
                    break;
                case 'n':
                case 'N':
                    continuousVolts = 0;
                    sprintf(str_buffer, "Continuous Volts OFF\n");
                    sendmsg(str_buffer);
                    break;
                case 'f':
                case 'F':
                    ServoFollowADC = 1;
                    /* Set the Servo mode to follow ADC0 RES */
                    break;
                case 'g':
                case 'G':
                    ServoFollowADC = 0;
                    /* Set the Servo mode move at a user selected speed */
                    break;
                case '0':
                    step_delay = 0;
                    break;
                case '1':
                    step_delay = 1000;
                    break;
                case '2':
                    step_delay = 750;
                    break;
                case '3':
                    step_delay = 500;
                    break;
                case '4':
                    step_delay = 400;
                    break;
                case '5':
                    step_delay = 250;
                    break;
                case '6':
                    step_delay = 200;
                    break;
                case '7':
                    step_delay = 150;
                    break;
                case '8':
                    step_delay = 100;
                    break;
                case '9':
                    step_delay = 50;
                    break;
                    /* Set the servomotor speed based on the specification table */
                    break;
                default:
                    sprintf(str_buffer, "Unrecognized input: %c\n", ch);
                    sendmsg(str_buffer);
                    break;
            }
        }
        if (continuousDistance && newDistanceData) {
	        newDistanceData = 0;
	        uint16_t distance_cm = pulseTime / 58;
	        sprintf(str_buffer, "Pulse Width = %u us\nDistance = %u cm\n", pulseTime, distance_cm);
	        sendmsg(str_buffer);
        }

        else if (continuousTime && newTimeData) {
	        newTimeData = 0;
	        if (periodTime > 0) {
		        uint16_t freq = 1000000UL / periodTime;
		        uint8_t duty = (pulseTime * 100UL) / periodTime;

		        sprintf(str_buffer,
		        "555 Timer:\nPeriod = %u us\nPulse = %u us\nFrequency = %u Hz\nDuty = %u%%\n",
		        periodTime, pulseTime, freq, duty);
		        sendmsg(str_buffer);
		        } else {
		        sendmsg("555 Timer: Invalid period\n");
	        }
        }
        else if (continuousVolts && newADC0Data) {
	        newADC0Data = 0;
	        uint16_t mV = ((uint32_t)adc_reading * 5000) / 1023;
	        sprintf(str_buffer, "ADC0 = %u (%u mV)\n", adc_reading, mV);
	        sendmsg(str_buffer);
        }
    }
}


/* sendmsg function */
void sendmsg(char* s) {
	 if (qcntr == sndcntr) {
		 qcntr = 0;
		sndcntr = 1;
		while (*s)
			queue[qcntr++] = *s++;
		USART3.TXDATAL = queue[0];
	 }
 }
 
 
 /* Function to set or clear all LED port bits, 1 - set, 0 - clear */
void Set_Clear_Ports(uint8_t set) {
 
	for (int i = 0; i < 10; i++)
	{
		if (set)
			LED_Array[i].LED_PORT->OUTSET = LED_Array[i].bit_mapping;
		else
			LED_Array[i].LED_PORT->OUTCLR = LED_Array[i].bit_mapping;
	}
 }
 
 
/* ****************************************************************/ 
/* Interrupt Service Routines */
/* ****************************************************************/  
ISR(TCB0_INT_vect)
 {
	TCB0.INTFLAGS = TCB_CAPT_bm; /* Clear the interrupt flag */
 
	/* Use this ISR to capture the HC-SR04 Pulse Width, which can be used 
	   to calculate the distance to an object */
	uint16_t pulseTime = TCB0.CCMP;  // Read the counter value at the capture momen
	
	newDistanceData = 1;
 }
 
 
 ISR(TCB2_INT_vect)
 {
	 uint16_t clocksP, clocksT, Pulse_Width, Time_Period;

	 // Clear the interrupt flag
	 TCB2.INTFLAGS = TCB_CAPT_bm;

	 /* Capture the Period and High Pulse Width number of clocks */
	 clocksT = TCB2.CNT;    // In PW - Freq mode, CNT stops on the trailing edge until CCMP is read.
	 clocksP = TCB2.CCMP;   // Capture the value when the pulse ends (on the trailing edge)

	 // Convert the captured counts to microseconds based on the system clock
	 Pulse_Width = clocksP / 10;   // Convert pulse width to microseconds
	 Time_Period = clocksT / 10;   // Convert time period to microseconds

	 // Set the New Input Capture data flag
	 newTimeData = 1;

	 // If the input period is greater than 300 us, turn on PORTB Pin 2 (LED bit 5)
	 if (Time_Period > 300) {
		 PORTB.OUTSET = (1 << 2);  // Turn on LED at PORTB Pin 2 (bit 2)
		 } else {
		 PORTB.OUTCLR = (1 << 2);  // Turn off LED at PORTB Pin 2 (bit 2)
	 }

	 // Always turn off PORTF Pin 4 (LED bit 6)
	 PORTF.OUTCLR = (1 << 4);  // Turn off LED at PORTF Pin 4 (bit 4)
 }

  
ISR(TCB3_INT_vect) {
	TCB3.INTFLAGS = TCB_CAPT_bm;  // Clear interrupt

	static uint8_t trigger_counter = 0;

	if (trigger_counter++ >= 10) {
		trigger_counter = 0;
		PORTB.OUTSET = PIN0_bm;   // Trigger pin high
		_delay_us(10);
		PORTB.OUTCLR = PIN0_bm;   // Trigger pin low
	}

	uint16_t step_delay = 500;  // Example integer delay value (in microseconds)

	if (ServoFollowADC) {
		// Map adc_reading (0–1023) to CMP0 value (1250–2500us)
		uint16_t cmp = 1250 + ((uint32_t)adc_reading * 1250 / 1023);
		TCA0.SINGLE.CMP0 = cmp;
		} else {
		if (servo_pos < NewServoPosition) {
			servo_pos++;
			_delay_us(step_delay);  // Now using integer for delay
			} else if (servo_pos > NewServoPosition) {
			servo_pos--;
			_delay_us(step_delay);  // Now using integer for delay
		}
		TCA0.SINGLE.CMP0 = servo_pos;
	}
}

 
 ISR(ADC0_RESRDY_vect)
 {
	 adc_reading = ADC0.RES;
	 
	 newADC0Data = 1;
	 
	 /* set the LED[7] on/off based on the adc_reading */
	 if (adc_reading > 716){
		 LED_Array[7].LED_PORT->OUTSET = LED_Array[7].bit_mapping;
	 } else {
		 LED_Array[7].LED_PORT->OUTCLR = LED_Array[7].bit_mapping;
	 }
	 
	 /* If ServoFollowADC == 1 set the servomotor position to a position based on the 
		adc_reading value */
	 if (ServoFollowADC) {
		// Map ADC0 result (0 to 1023) to an angle between -90° and +90°
		int16_t angle = ((adc_reading * 180) / 1023) - 90;

		// Now map the angle (-90° to +90°) to a servo pulse width (1000 µs to 2000 µs)
		uint8_t servo_pulse = 1500 + ((angle * 500) / 90);  // 1500 is the neutral pulse width (centered position)

		// Set the servo position using TCA0.SINGLE.CMP0BUF
		TCA0.SINGLE.CMP0BUF = servo_pulse;
	}
}
 


ISR(USART3_TXC_vect) {
	  USART3.STATUS |= USART_TXCIF_bm;
	  if (qcntr != sndcntr)
	  USART3.TXDATAL = queue[sndcntr++];
 }

