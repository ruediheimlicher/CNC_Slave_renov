//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 14.10.07.
//  Copyright Ruedi Heimlicher 2007. All rights reserved.
//


#include <string.h>
#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>

#include "lcd.c"
#include "adc.c"
#include "version.c"
#include "usb_rawhid.c"
//#include "ringbuffer.c"

// USB
#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))


volatile uint8_t do_output=0;
static volatile uint8_t buffer[32]={};
static volatile uint8_t sendbuffer[32]={};

// begin Ringbuffer
#define RINGBUFFERTIEFE 4
#define READYBIT   0       // buffer kann Daten aufnehmen
#define FULLBIT   1        // Buffer ist voll
#define STARTBIT   2       // Buffer ist geladen
#define RINGBUFFERBIT 3    // Ringbuffer wird verwendet
#define LASTBIT   4         // Letzter Abschnitt  ist geladen
#define ENDBIT   5          // Letzter Abschnitt  ist abgearbeitet
#define STOPBIT   6        // Ablauf stoppen
#define FIRSTBIT   7

uint8_t CNCDaten[RINGBUFFERTIEFE][33];
uint8_t CDCStringArray[RINGBUFFERTIEFE];

//volatile uint8_t inposition= 0;
//volatile uint8_t outposition= 0;

volatile uint16_t abschnittnummer=0;
volatile uint16_t endposition= 0xFFFF;
volatile uint8_t ladeposition=0;

volatile uint8_t ringbufferstatus=0x00;   

uint16_t AbschnittCounter=0;
volatile uint8_t liniencounter= 0;
// end Ringbuffer
// end USB 

//#define SDAPIN		4
//#define SCLPIN		5

// SPI
#define OSZIPORT	PORTD
#define OSZIPORTDDR	DDRD
#define OSZIPORTPIN	PIND
#define OSZI_PULS_A	0
#define OSZI_PULS_B	1

#define OSZI_A_LO OSZIPORT &= ~(1<<OSZI_PULS_A)
#define OSZI_A_HI OSZIPORT |= (1<<OSZI_PULS_A)


#define OSZI_B_LO OSZIPORT &= ~(1<<OSZI_PULS_B)
#define OSZI_B_HI OSZIPORT |= (1<<OSZI_PULS_B)
#define OSZI_B_TOGG OSZIPORT ^= (1<<OSZI_PULS_B)
// SPI



#define TIMER0_STARTWERT	0x40

#define LOOPLEDDDR          DDRF    //DDRD
#define LOOPLEDPORT         PORTF   //PORTD
#define LOOPLED             4       //6 

#define TASTENDDR           DDRF
#define TASTENPORT          PORTF
#define TASTENPIN          PINF

#define TASTE0				0   // HALT-PIN Motor A
#define TASTE1				1


#define STARTDELAYBIT       0
#define HICOUNTBIT          1

#define WDTBIT              7

// CNC12
#define CMD_PORT            PORTD   //    PORTB
#define CMD_DDR             DDRD    //    DDRB
#define CMD_PIN             PIND    //    PINB

#define LOAD_NEXT          5  // Bit fuer Laden des naechsten Abschnitts in der loop
#define LOAD_LAST          6  // Bit fuer Laden des letzten Abschnitts in der loop


#define END_A0_PIN          0       //  Endanschlag A0 
#define END_B0_PIN          1 		//  Endanschlag A1 


#define RICHTUNG_A	0
#define RICHTUNG_B	1
#define RICHTUNG_C	2
#define RICHTUNG_D	3

// Seite 1

#define STEPPERPORT_1	PORTC
#define STEPPERDDR_1    DDRC
#define STEPPERPIN_1    PINC

#define MA_STEP         0
#define MA_RI           1
#define MA_EN           2

#define MB_STEP         3
#define MB_RI           4
#define MB_EN           5

#define END_A0          6           // Bit fuer Endanschlag bei A0
#define END_B0          7           // Bit fuer Endanschlag bei A1


// Seite 2

#define STEPPERPORT_2      PORTB
#define STEPPERDDR_2       DDRB
#define STEPPERPIN_2       PINB

#define MC_STEP            0           // PIN auf Stepperport 2
#define MC_RI              1
#define MC_EN              2

#define MD_STEP            3           // PIN auf Stepperport 2
#define MD_RI              4
#define MD_EN              5

#define END_C0             6           // Anschlagstatus:  Bit fuer Endanschlag bei C0
#define END_D0             7           // Anschlagstatus:  Bit fuer Endanschlag bei D0


#define HALT_PIN 0

#define COUNT_A				4		// Motorstatus:   Schritte von Motor A zaehlen
#define COUNT_B				5		// Motorstatus:   Schritte von Motor B zaehlen

#define COUNT_C				4		// Motorstatus:   Schritte von Motor C zaehlen
#define COUNT_D				5		// Motorstatus:   Schritte von Motor D zaehlen



#define DC                  7    // DC ON: LO
#define STROM               4    // Stepperstrom ON: HI

#define GO_HOME            3     // Bit fuer befehl beginn home auf cncstatus
#define DC_DIVIDER         1      // teilt die pwm-Frequenz in ISR

volatile uint8_t timer0startwert=TIMER0_STARTWERT;
#define USB_DATENBREITE 32
//volatile uint8_t rxbuffer[USB_DATENBREITE];

/*Der Sendebuffer, der vom Master ausgelesen werden kann.*/
//volatile uint8_t txbuffer[USB_DATENBREITE];

//uint16_t EEMEM Brennerlaufzeit;	// Akkumulierte Laufzeit

void delay_ms(unsigned int ms);

volatile uint8_t           cncstatus=0x00;

volatile uint8_t           usbstatus=0x00;
static volatile uint8_t    motorstatus=0x00;
static volatile uint8_t    anschlagstatus=0x00;

#define USB_SEND  0 

volatile uint8_t status=0;

volatile uint8_t           PWM=0;
static volatile uint8_t    pwmposition=0;
static volatile uint8_t    pwmdivider=0;


volatile char SPI_data='0';
volatile char SPI_dataArray[32];

// CNC

volatile uint16_t CounterA=0;			// Zaehler fuer Delay von Motor A 
volatile uint16_t CounterB=0;			// Zaehler fuer Delay von Motor B
volatile uint16_t CounterC=0;			// Zaehler fuer Delay von Motor C 
volatile uint16_t CounterD=0;			// Zaehler fuer Delay von Motor D

volatile uint16_t DelayA=24;			// Delay von Motor A 
volatile uint16_t DelayB=24;			// Delay von Motor B 
volatile uint16_t DelayC=24;			// Delay von Motor C 
volatile uint16_t DelayD=24;			// Delay von Motor D 

volatile uint16_t StepCounterA=0;	// Zaehler fuer Schritte von Motor A 
volatile uint16_t StepCounterB=0;	// Zaehler fuer Schritte von Motor B
volatile uint16_t StepCounterC=0;	// Zaehler fuer Schritte von Motor C 
volatile uint16_t StepCounterD=0;	// Zaehler fuer Schritte von Motor D

volatile uint8_t richtung=0;

void slaveinit(void)
{
	//OSZIPORTDDR |= (1<<PULS);	// Output
	//OSZIPORT |= (1<<PULS);		// HI
	
	LOOPLEDDDR |=(1<<LOOPLED);
	LOOPLEDPORT |= (1<<LOOPLED);	// HI

	STEPPERDDR_1 |= (1<<MA_STEP);
	STEPPERPORT_1 |= (1<<MA_STEP);	// HI
	
	STEPPERDDR_1 |= (1 << MA_RI);
	STEPPERPORT_1 |= (1 << MA_RI);	// HI
	STEPPERDDR_1 |= (1 << MA_EN);
	STEPPERPORT_1 &= ~(1 << MA_EN);   // LO
	STEPPERPORT_1 |= (1 << MA_EN);	// HI
	
	STEPPERDDR_1 |= (1 << MB_STEP);
	STEPPERPORT_1 |= (1 << MB_STEP); // HI
	
	STEPPERDDR_1 |= (1 << MB_RI);
	STEPPERPORT_1 |= (1 << MB_RI);	// HI
	
	STEPPERDDR_1 |= (1 << MB_EN);
	STEPPERPORT_1 &= ~(1 << MB_EN); // LO
   
   //Seite 2
	STEPPERDDR_2 |= (1<<MC_STEP);
	STEPPERPORT_2 |= (1<<MC_STEP);	// HI
	
	STEPPERDDR_2 |= (1 << MC_RI);
	STEPPERPORT_1 |= (1 << MC_RI);	// HI
   
	STEPPERDDR_2 |= (1 << MC_EN);
	STEPPERPORT_2 |= (1 << MC_EN);	// HI
	
	STEPPERDDR_2 |= (1 << MD_STEP);
	STEPPERPORT_2 |= (1 << MD_STEP); // HI
	
	STEPPERDDR_2 |= (1 << MD_RI);
	STEPPERPORT_2 |= (1 << MD_RI);	// HI
	
	STEPPERDDR_2 |= (1 << MD_EN);
   STEPPERPORT_2 |= (1 << MD_EN);	// HI
   
	
	//Pin 0 von   als Ausgang fuer OSZI
	OSZIPORTDDR |= (1<<OSZI_PULS_A);	//Pin 0 von  als Ausgang fuer LED TWI
    OSZIPORT |= (1<<OSZI_PULS_A);		// HI
	
    OSZIPORTDDR |= (1<<OSZI_PULS_B);		//Pin 1 von  als Ausgang fuer LED TWI
    OSZIPORT |= (1<<OSZI_PULS_B);		//Pin   von   als Ausgang fuer OSZI
	

	TASTENDDR &= ~(1<<TASTE0);	//Bit 0 von PORT B als Eingang fŸr Taste 0
	TASTENPORT |= (1<<TASTE0);	//Pull-up

//	DDRB &= ~(1<<PORTB1);	//Bit 1 von PORT B als Eingang fŸr Taste 1
//	PORTB |= (1<<PORTB1);	//Pull-up
	

	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);		//Pin 4 von PORT D als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 5 von PORT D als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 6 von PORT D als Ausgang fuer LCD

/*
	DDRC &= ~(1<<PORTC4);	//Bit 4 von PORT C als Eingang fŸr SDA
	PORTC |= (1<<PORTC4);	//Pull-up
	DDRC &= ~(1<<PORTC5);	//Bit 5 von PORT C als Eingang fŸr SCL
	PORTC |= (1<<PORTC5);	//Pull-up
*/	
	//Versuch mit init von CNC 12
//	CMD_DDR &= ~(1<<END_A0_PIN);			//	Bit 0 von PORT B als Eingang fŸr Endanschlag A0
//	CMD_PORT |= (1<<END_A0_PIN);			// Pull-up

//   CMD_DDR &= ~(1<<END_B0_PIN);			//	Bit 1 von PORT B als Eingang fŸr Endanschlag A1
//	CMD_PORT |= (1<<END_B0_PIN);			// Pull-up

//	CMD_DDR &= ~(1<<PORTB1);			// Bit 1 von PORT B als Eingang fŸr Taste 1
//	CMD_PORT |= (1<<PORTB1);			//	Pull-up
	
//   DDRD |= (1<<PORTD6);
 //  PORTD |= (1<<PORTD6);
   CMD_DDR |= (1<<DC);                       // DC-PWM-Ausgang
   CMD_PORT &= ~(1<<DC);                      // LO
   
   CMD_DDR |= (1<<STROM);                    // Stepperstrom-Ausgang, Active HI
   CMD_PORT &= ~(1<<STROM);                  // LO
   CMD_PORT |= (1<<STROM);
}



void delay_ms(unsigned int ms)/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms){
		_delay_ms(0.96);
		ms--;
	}
}

void timer0 (void) 
{ 
// Timer fuer Exp
	//TCCR0 |= (1<<CS01);						// clock	/8
	//TCCR0 |= (1<<CS01)|(1<<CS02);			// clock	/64
	TCCR0B |= (1<<CS02)| (1<<CS02);			// clock	/256
	//TCCR0 |= (1<<CS00)|(1<<CS02);			// clock /1024
	TCCR0B |= (1 << CS10); // Set up timer 
	OCR0A = 0x2;
	
	//TIFR |= (1<<TOV0);							//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK0 |= (1<<TOIE0);							//Overflow Interrupt aktivieren
	TCNT0 = TIMER0_STARTWERT;					//RŸcksetzen des Timers

}

/*
void timer2 (uint8_t wert) 
{ 
//	TCCR2 |= (1<<CS02);							//8-Bit Timer, Timer clock = system clock/256

//Takt fuer Servo
	TCCR2 |= (1<<CS20)|(1<<CS21);				//Takt /64	Intervall 64 us

	TCCR2 |= (1<<WGM21);							//	ClearTimerOnCompareMatch CTC

	//OC2 akt
//	TCCR2 |= (1<<COM20);						//	OC2 Pin zuruecksetzen bei CTC


	TIFR |= (1<<TOV2);							//Clear TOV2 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK0 |= (1<<OCIE2);							//CTC Interrupt aktivieren

	TCNT2 = 0x00;									//Zaehler zuruecksetzen
	
	OCR2 = wert;									//Setzen des Compare Registers auf Servoimpulsdauer
} 
*/

volatile uint16_t timer2Counter=0; 

ISR (TIMER2_OVF_vect) 
{ 
	timer2Counter +=1;
	
	if (timer2Counter >= 14) 
	{
		CounterA-=1;
		CounterB-=1;
		timer2Counter = 0; 
        //OSZI_B_TOGG ;
	} 
	TCNT2 = 10;							// ergibt 2 kHz fuer Timertakt
}

/*
ISR(TIMER2_COMP_vect) // Schaltet Impuls an SERVOPIN0 aus
{
//		lcd_clr_line(1);
//		lcd_puts("Timer2 Comp\0");
		TCCR20=0;
}
*/

void DatenLadenAnPosition(uint16_t diePosition)
{
   
   

}


//uint8_t  AbschnittLadenVonPosition(uint16_t diePosition)


uint8_t  AbschnittLaden_4M(const uint8_t* AbschnittDaten) // 22us
{
   
	uint8_t returnwert=0;
	/*			
	 Reihenfolge der Daten:
    0    schritteax lb
    1    schritteax hb
    2    schritteay lb
    3    schritteay hb
    
    4    delayax lb
    5    delayax hb
    6    delayay lb
    7    delayay hb
    
    8    schrittebx lb
    9    schrittebx hb
    10    schritteby lb
    11    schritteby hb
    
    12    delaybx lb
    13    delaybx hb
    14    delayby lb
    15    delayby hb
    
    
    16   (8)    code
    17   (9)    position // Beschreibung der Lage im Schnittpolygon:first, last, ...
    18   (10)   indexh     // Nummer des Abschnitts
    19   (11)   indexl   
    
    20     pwm
    
    21   motorstatus // relevanter Motor fuer Abschnitt
	 */			
	int lage = 0;
   //   lage = AbschnittDaten[9]; // Start: 1, innerhalb: 0, Ende: 2
   lage = AbschnittDaten[17]; // Start: 1, innerhalb: 0, Ende: 2
	if (lage & 0x01)
   {
      returnwert=1;
   }
   if (lage & 0x02) // letztes Element
   {
      returnwert=2;
   }
   richtung=0;
   
	// Motor A
	STEPPERPORT_1 &= ~(1<<MA_EN); // Pololu ON
	
	uint8_t dataL=0;
	uint8_t dataH=0;
	
	uint8_t delayL=0;
	uint8_t delayH=0;
	
	dataL=AbschnittDaten[0];
	dataH=AbschnittDaten[1];
	
	//lcd_gotoxy(17,0);
	if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
	{
		richtung |= (1<<RICHTUNG_A); // Rueckwarts
		STEPPERPORT_1 &= ~(1<< MA_RI); // PIN fuer Treiber stellen
		//lcd_putc('r');
	}
	else 
	{
		richtung &= ~(1<<RICHTUNG_A);
		STEPPERPORT_1 |= (1<< MA_RI);
		//lcd_putc('v');	// Vorwaerts
	}
	
	dataH &= (0x7F);
	StepCounterA = dataH;		// HByte
	StepCounterA <<= 8;        // shift 8
	StepCounterA += dataL;     // +LByte
	
	delayL=AbschnittDaten[4];
	delayH=AbschnittDaten[5];
	
	
	DelayA = delayH;
	DelayA <<= 8;
	DelayA += delayL;
	
   CounterA = DelayA;
   
	// Motor B
	//CounterB=0;
	STEPPERPORT_1 &= ~(1<<MB_EN);	// Pololu ON
	dataL=AbschnittDaten[2];
	dataH=AbschnittDaten[3];
	//lcd_gotoxy(19,1);
   
	if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
	{
		richtung |= (1<<RICHTUNG_B); // Rueckwarts
		STEPPERPORT_1 &= ~(1<< MB_RI);
		//lcd_putc('r');
	}
	else 
	{
		richtung &= ~(1<<RICHTUNG_B);
		STEPPERPORT_1 |= (1<< MB_RI);
		//lcd_putc('v');
	}
	
	dataH &= (0x7F);
	StepCounterB = dataH;		// HByte
	StepCounterB <<= 8;		// shift 8
	StepCounterB += dataL;	// +LByte
	
	DelayB = (AbschnittDaten[7]<<8)+ AbschnittDaten[6];
   
   CounterB = DelayB;
   
	// Motor C
	STEPPERPORT_2 &= ~(1<<MC_EN); // Pololu ON
	//CounterC=0;
	dataL=0;
	dataH=0;
	
	delayL=0;
	delayH=0;
	
	dataL=AbschnittDaten[8];
	dataH=AbschnittDaten[9];
   
	//richtung=0;
	if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
	{
		richtung |= (1<<RICHTUNG_C); // Rueckwarts
		STEPPERPORT_2 &= ~(1<< MC_RI);
	}
	else 
	{
		richtung &= ~(1<<RICHTUNG_C);
		STEPPERPORT_2 |= (1<< MC_RI);
	}
	
	dataH &= (0x7F);
	StepCounterC = dataH;		// HByte
	StepCounterC <<= 8;		// shift 8
	StepCounterC += dataL;	// +LByte
   
	
	delayL=AbschnittDaten[12];
	delayH=AbschnittDaten[13];
   
	DelayC = delayH;
	DelayC <<=8;
	DelayC += delayL;
   
   CounterC = DelayC;
   
   // Motor D
	STEPPERPORT_2 &= ~(1<<MD_EN); // Pololu ON
	//CounterD=0;
	dataL=0;
	dataH=0;
	
	delayL = 0;
	delayH = 0;
	
	dataL = AbschnittDaten[10];
	dataH = AbschnittDaten[11];
   
	if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
	{
		richtung |= (1<<RICHTUNG_D); // Rueckwarts
		STEPPERPORT_2 &= ~(1<< MD_RI);
		//lcd_putc('r');
	}
	else 
	{
		richtung &= ~(1<<RICHTUNG_D);
		STEPPERPORT_2 |= (1<< MD_RI);
	}
	
	dataH &= (0x7F);
	StepCounterD= dataH;		// HByte
	StepCounterD <<= 8;		// shift 8
	StepCounterD += dataL;	// +LByte
	
	delayL=AbschnittDaten[14];
	delayH=AbschnittDaten[15];
   
	DelayD = delayH;
	DelayD <<= 8;
	DelayD += delayL;
   
   CounterD = DelayD;
   
   // pwm-rate
   PWM = AbschnittDaten[20];
   
   // motorstatus: welcher Motor ist relevant
   motorstatus = AbschnittDaten[21];
   return returnwert;
   
   // Nicht mehr verwendet, wird in Stepper berechnet
   if (StepCounterA > StepCounterB) 
   {
      if (StepCounterA > StepCounterC)
      {
         if (StepCounterA > StepCounterD) // A max
         {
            motorstatus |= (1<<COUNT_A);
            //lcd_putc('A');
         }
         else //A>B A>C D>A
         {
            motorstatus |= (1<<COUNT_D);
            //lcd_putc('D');
         }
         
      }//A>C
      else // A>B A<C: A weg, B weg
      {
         if (StepCounterC > StepCounterD)
         {
            motorstatus |= (1<<COUNT_C);
            //lcd_putc('C');
         }
         else // A>B A<C D>C B weg, 
         {
            motorstatus |= (1<<COUNT_D);
            //lcd_putc('D');
         }
         
         
      }
   }// A>B
   
   
   else // B>A A weg
   {
      if (StepCounterB > StepCounterC) // C weg
      {
         if (StepCounterB > StepCounterD) // D weg
         {
            motorstatus |= (1<<COUNT_B);
            //lcd_putc('B');
         }
         else
         {
            motorstatus |= (1<<COUNT_D);
            //lcd_putc('D');
         }
      }
      else // B<C B weg
      {  
         if (StepCounterC > StepCounterD) // D weg
         {
            motorstatus |= (1<<COUNT_C);
            //lcd_putc('C');
         }
         else // D>C C weg
         {
            motorstatus |= (1<<COUNT_D);
            //lcd_putc('D');
         }
         
      }
   }
   
   //OSZI_A_HI;
   return returnwert;
}

uint8_t  AbschnittLaden(const uint8_t* AbschnittDaten)
{
	uint8_t returnwert=0;
	/*			
	 Reihenfolge der Daten:
	 schrittexl"
	 schrittexh"
	 schritteyl"
	 schritteyh"
	 delayxl"
	 delayxh"
	 delayyl"
	 delayyh"
	 code"
    position   Beschreibung der Lage im Schnittpolygon: first, last
    
    
	 */			
	int lage = 0;
   lage = AbschnittDaten[9]; // Start: 1, innerhalb: 0, Ende: 2
	if (lage & 0x01)
   {
      returnwert=1;
   }
   if (lage & 0x02) // letztes Element
   {
      returnwert=2;
   }
   
   
	// Motor A
	STEPPERPORT_1 &= ~(1<<MA_EN); // Pololu ON
	
//   STEPPERPORT_1 &= ~(1<<MC_EN); // Pololu ON
   
   CounterA=0;
	uint8_t dataL=0;
	uint8_t dataH=0;
	
	uint8_t delayL=0;
	uint8_t delayH=0;
	
	dataL=AbschnittDaten[0];
	dataH=AbschnittDaten[1];
	
	//lcd_gotoxy(18,0);
	if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
	{
		richtung |= (1<<RICHTUNG_A); // Rueckwarts
		STEPPERPORT_1 &= ~(1<< MA_RI);
      
		//lcd_putc('r');
	}
	else 
	{
		richtung &= ~(1<<RICHTUNG_A);
		STEPPERPORT_1 |= (1<< MA_RI);
		//lcd_putc('v');	// Vorwaerts
	}
	
	dataH &= (0x7F);
	StepCounterA= dataH;		// HByte
	StepCounterA <<= 8;		// shift 8
	StepCounterA += dataL;	// +LByte
	
	delayL=AbschnittDaten[4];
	delayH=AbschnittDaten[5];
	
	
	DelayA = delayH;
	DelayA <<=8;
	DelayA += delayL;
   
   CounterA = DelayA;
   
   CounterC = DelayA;
	
	/*
    lcd_gotoxy(0,0);
    lcd_putc('A');
    lcd_putc(' ');
    lcd_putc('s');
    lcd_putint(dataH);
    lcd_putc(' ');
    lcd_putint(dataL);
    
    lcd_putc(' ');
    lcd_putc('d');
    lcd_putint (delayH);
    lcd_putint(delayL);
    
    */
	// Motor B
	CounterB=0;
	//STEPPERPORT_1 |= (1<<MB_EN);
	STEPPERPORT_1 &= ~(1<<MB_EN);	// Pololu ON
   
   
   
	dataL=AbschnittDaten[2];
	dataH=AbschnittDaten[3];
	//lcd_gotoxy(19,1);
   
	if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
	{
		richtung |= (1<<RICHTUNG_B); // Rueckwarts
		STEPPERPORT_1 &= ~(1<< MB_RI);
		//lcd_putc('r');
	}
	else 
	{
		richtung &= ~(1<<RICHTUNG_B);
		STEPPERPORT_1 |= (1<< MB_RI);
		//lcd_putc('v');
	}
	
	dataH &= (0x7F);
	StepCounterB= dataH;		// HByte
	StepCounterB <<= 8;		// shift 8
	StepCounterB += dataL;	// +LByte
	
	DelayB = (AbschnittDaten[7]<<8)+ AbschnittDaten[6];
   
   CounterB = DelayB;
   
   if (StepCounterA > StepCounterB) // Hoeherer Wert setzt relevante Zaehlvariable
   {
      cncstatus |= (1 << COUNT_A); // Schritte von Motor A zaehlen 
      cncstatus &= ~(1 << COUNT_B);// Bit von Motor B zuruecksetzen
      //lcd_putc('A');
   }
   else
   {
      cncstatus |= (1 << COUNT_B); // Schritte von Motor B zaehlen 
      cncstatus &= ~(1 << COUNT_A);// Bit von Motor A zuruecksetzen
      //lcd_putc('B');
   }
   
	/*
	 lcd_gotoxy(0,1);
	 lcd_putc('B');
	 lcd_putc(' ');
	 lcd_putc('s');
	 lcd_putint(dataH);
	 //lcd_putc(' ');
	 lcd_putint(dataL);
	 
	 lcd_putc(' ');
	 lcd_putc('d');
	 lcd_putint (rxbuffer[7]);
	 lcd_putint(rxbuffer[6]);
	 */
   return returnwert;
}



void AbschnittEndVonMotor(const uint8_t derstatus) // 0 - 3 fuer A - D   52 us
{
   uint8_t motor = (derstatus & 0x30)>>4;
   //uint8_t motor = derstatus ;
   //motor=0;
   //   STEPPERPORT_1 |= (1<<(MA_EN + motor));					// Motor A... OFF
   
   
   if (motor < 2)
   {
      //    STEPPERPORT_1 |= (1<<(MA_EN + motor));
      //    StepCounterA=0;
      //    StepCounterB=0;
   }
   else
   {
      //   STEPPERPORT_2 |= (1<<(MA_EN + motor -2)); // MC_EN ist = MA_EN, aber motor ist 3
      //   StepCounterC=0;
      //   StepCounterD=0;
      
   }
   
   sendbuffer[16]=StepCounterA & 0xFF;
   sendbuffer[17]=StepCounterB & 0xFF;
   sendbuffer[18]=StepCounterC & 0xFF;
   sendbuffer[19]=StepCounterD & 0xFF;
   sendbuffer[20]=derstatus;
   sendbuffer[21]=motor;
   sendbuffer[22]=motorstatus;
   
   //   STEPPERPORT_1 |= (1<<(MA_EN + motor));
   if (StepCounterA)
   {
      StepCounterA=1;
      
   }
   StepCounterA=0;
   
   if (StepCounterB)
   {
      StepCounterB=1;
   }
   
   StepCounterB=0;
   
   CounterA=0xFFFF;
   //CounterA=0;
   
   CounterB=0xFFFF;
   //CounterB=0;
   
   //   STEPPERPORT_2 |= (1<<(MA_EN + motor -2));
   if (StepCounterC)
   {
      StepCounterC=1;
   }
   StepCounterC=0;
   
   if (StepCounterD)
   {
      StepCounterD=1;
   }
   StepCounterD=0;
   
   CounterC=0xFFFF;
   //CounterC=0;
   
   CounterD=0xFFFF;
   //CounterD=0;
   
   OSZI_B_LO;
   if (abschnittnummer==endposition) // Serie fertig
   {  
      ringbufferstatus = 0;
      //anschlagstatus=0;
      anschlagstatus &= ~(1<< (END_A0 + motor)); // Bit fuer Anschlag Ax zuruecksetzen
      motorstatus=0;
      //sendbuffer[0]=0xAA + motor;
      sendbuffer[0]=0xAA + motor;
      
      sendbuffer[1]=abschnittnummer;
      sendbuffer[5]=abschnittnummer;
      sendbuffer[6]=ladeposition;
      sendbuffer[7]=cncstatus;
      usb_rawhid_send((void*)sendbuffer, 50);
      sei();

      ladeposition=0;
      
      STEPPERPORT_1 |= (1<<MA_EN);
      STEPPERPORT_1 |= (1<<MB_EN);
      STEPPERPORT_2 |= (1<<MC_EN);
      STEPPERPORT_2 |= (1<<MD_EN);
   }
   else 
   { 
      
      uint8_t aktuellelage=0;
      {
         uint8_t aktuelleladeposition=(ladeposition & 0x00FF);
         aktuelleladeposition &= 0x03; // Position im Ringbuffer
         // aktuellen Abschnitt laden
         
         aktuellelage = AbschnittLaden((uint8_t*)CNCDaten[aktuelleladeposition]); //gibt Lage zurueck: 1: Anfang, 2: Ende, 0; innerhalb
         uint8_t aktuellermotor = motor;
         aktuellermotor <<=6;
         cncstatus |= aktuellermotor;
         
         if (aktuellelage==2) // war letzter Abschnitt
         {
            endposition=abschnittnummer; // letzter Abschnitt zu fahren
            
            cncstatus |= (1<<LOAD_LAST);
            
            // Neu: letzen Abschnitt melden
            sendbuffer[0]=0xD0;
            sendbuffer[5]=abschnittnummer;
            sendbuffer[6]=ladeposition;
            usb_rawhid_send((void*)sendbuffer, 50);
            sei();
            // end neu
         }  
         else
         {
            cncstatus |= (1<<LOAD_NEXT);
            // neuen Abschnitt abrufen
            
            sendbuffer[5]=abschnittnummer;
            sendbuffer[6]=ladeposition;
            sendbuffer[0]=0xA0 + motor;
            usb_rawhid_send((void*)sendbuffer, 50);
            sei();
         }
         
         ladeposition++; // Position im Ringbuffer
         
      }
      AbschnittCounter++;
      //OSZI_A_LO;
      OSZI_B_HI;
   }
   
}

uint8_t RingbufferLaden(const uint8_t outpos )
{
   uint8_t lage=0;
   return lage;
}

int main (void) 
{
    int8_t r;

uint16_t count=0;
    
	// set for 16 MHz clock
	CPU_PRESCALE(0);
    
	// Initialize the USB, and then wait for the host to set configuration.
	// If the Teensy is powered without a PC connected to the USB port,
	// this will wait forever.
	usb_init();
	while (!usb_configured()) /* wait */ ;
    
	// Wait an extra second for the PC's operating system to load drivers
	// and do whatever it does to actually be ready for input
	_delay_ms(1000);

	//in Start-loop in while
	//init_twi_slave (SLAVE_ADRESSE);
	sei();
	
	
	slaveinit();
		
	/* initialize the LCD */
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);

	lcd_puts("Guten Tag\0");
	delay_ms(100);
	lcd_cls();
	//lcd_puts("READY\0");
	lcd_puts("V: \0");
	lcd_puts(VERSION);
   lcd_clr_line(1);

	uint8_t Tastenwert=0;
	uint8_t TastaturCount=0;
	
	uint16_t TastenStatus=0;
	uint16_t Tastencount=0;
	uint16_t Tastenprellen=0x01F;
	//timer0();
	
	//initADC(TASTATURPIN);
	//wdt_enable(WDTO_2S);
	
	uint16_t loopcount0=0;
	uint8_t loopcount1=0;

	
	
	
	/*
	Bit 0: 1 wenn wdt ausgelšst wurde
	 
	  */ 
	uint8_t i=0;
   
   //sendbuffer = malloc(64);
	//timer2
	//DDRD |= 1 << DDD1; // set LED pin PD1 to output 
	TCNT2   = 0; 
//	TCCR2A |= (1 << WGM21); // Configure timer 2 for CTC mode 
	TCCR2B |= (1 << CS20); // Start timer at Fcpu/8 
//	TIMSK2 |= (1 << OCIE2A); // Enable CTC interrupt 
	TIMSK2 |= (1 << TOIE2); // Enable OV interrupt 
	//OCR2A   = 5; // Set CTC compare value with a prescaler of 64 
    TCCR2A = 0x00;

	  sei();
	  
#pragma mark while	  
	while (1)
	{	
      
      //OSZI_B_LO;
		//Blinkanzeige
		loopcount0+=1;
		if (loopcount0==0xAFFF)
		{
			loopcount0=0;
			loopcount1+=1;
			LOOPLEDPORT ^=(1<<LOOPLED);
         //PORTD ^= (1<<PORTD6);
			//
			//STEPPERPORT_1 ^= (1 << MA_STEP);
			//PORTD ^= (1<<0);
			//lcd_gotoxy(18,1);
			//lcd_puthex(loopcount1);
			//timer0();
		}
		
       /**	Begin USB-routinen	***********************/
      
        // Start USB
      //lcd_putc('u');
      r = usb_rawhid_recv((void*)buffer, 0);
		if (r > 0) 
      {
         //OSZI_B_HI;
         cli(); 
         
         {
            //abschnittnummer=0;
            uint8_t indexh=buffer[10];
            uint8_t indexl=buffer[11];
            abschnittnummer= indexh<<8;
            abschnittnummer += indexl;
            //lcd_gotoxy(0,0);
            //lcd_putint2(indexh);
            //lcd_putint2(indexl);
            sendbuffer[0]=0x00;
            sendbuffer[5]=0x00;
            sendbuffer[6]=0x00;

            usb_rawhid_send((void*)sendbuffer, 50);
            sendbuffer[0]=0x00;
            sendbuffer[5]=0x00;
            sendbuffer[6]=0x00;


            
               
               if (abschnittnummer==0)
               {
                  //lcd_clr_line(0);
                  //lcd_clr_line(1);
                  //lcd_putc('a'+ liniencounter++);
                  //lcd_puthex(endposition);
                  //lcd_putint2(abschnittnummer);
                  cli();
                  ladeposition=0;
                  endposition=0xFFFF;
                  cncstatus=0;
                  ringbufferstatus=0x00;
                  //lcd_puthex(ladeposition);
                  //lcd_puthex(endposition);
                  //lcd_puthex(ringbufferstatus);
                  ringbufferstatus |= (1<<FIRSTBIT);
                  AbschnittCounter=0;
                  //lcd_gotoxy(0,0);
                  
                  sei();
                  
               }
               else
               {
                  
               }
               
               if (buffer[9]& 0x02)// letzter Abschnitt
               {
                  //lcd_gotoxy(3,0);
                  //lcd_putc('e');
                  //lcd_putint2(indexl);
                  ringbufferstatus |= (1<<LASTBIT);
                  if (ringbufferstatus & (1<<FIRSTBIT)) // nur ein Abschnitt
                      {
                         endposition=abschnittnummer; // erster ist letzter Abschnitt
                      }
               }
               
               
              
               // Daten vom buffer in CNCDaten laden
               {
                  uint8_t pos=(abschnittnummer);
                  pos &= 0x03; // 2 bit // Beschraenkung des index auf Buffertiefe 
                  if (abschnittnummer>8)
                  {
                     //lcd_putint1(pos);
                  }
                  uint8_t i=0;
                  for(i=0;i<USB_DATENBREITE;i++)
                  {
                      CNCDaten[pos][i]=buffer[i];  
                  }
                  
               }
               
               
               // Erster Abschnitt, naechsten Abschnitt laden
               if ((abschnittnummer == 0)&&(endposition))
               {
                  {
                     //lcd_putc('*');
                     sendbuffer[5]=abschnittnummer;
                     sendbuffer[6]=ladeposition;
                     sendbuffer[0]=0xA1;
                     usb_rawhid_send((void*)sendbuffer, 50);
                     sendbuffer[0]=0x00;
                     sendbuffer[5]=0x00;
                     sendbuffer[6]=0x00;

                     
                  }  
               }
               
               ringbufferstatus &= ~(1<<FIRSTBIT);
               
               // Ringbuffer ist voll oder  letzter Abschnitt schon erreicht
                           //if ((abschnittnummer >= 2)||(ringbufferstatus & (1<<LASTBIT)))                {
               if ((abschnittnummer ==1 )||((abschnittnummer ==0 )&&(ringbufferstatus & (1<<LASTBIT)))) 
               {
                   {
                      ringbufferstatus &= ~(1<<LASTBIT);
                     //Beginnen
                      //lcd_putc('s');
                     ringbufferstatus |= (1<<STARTBIT);
                      
                  }
               }
            
         } 
         sei();
		} // r>0, neue Daten
      
      /**	End USB-routinen	***********************/
		/**	Start CNC-routinen	***********************/
      
      if (ringbufferstatus & (1<<STARTBIT)) // Buffer ist geladen, Abschnitt 0 laden
      {
         cli();
         //lcd_putc('g'); // los
         ringbufferstatus &= ~(1<<STARTBIT);         
         ladeposition=0;
         AbschnittCounter=0;
         // Ersten Abschnitt laden
         uint8_t pos=AbschnittLaden(CNCDaten[0]); 
         ladeposition++;
         if (pos==2) // nur ein Abschnitt
         {
            ringbufferstatus |=(1<<ENDBIT);
         }
         
         AbschnittCounter+=1;
         sei();
      }
       
      /*
      // Anschlag A0
        if ((CMD_PIN & (1<< END_A0_PIN)) ) // Schlitten nicht am Anschlag A0
        {
            cncstatus &= ~(1<< END_A0); // Bit fuer Anschlag A0 zuruecksetzen
        }
        else                            // Schlitten am Anschlag A0
        {
            if (richtung & (1<<RICHTUNG_A)) // Richtung ist von Anschlag A0 weg, nichts tun
            {
               //lcd_putc(' ');
                cncstatus &= ~(1<< END_A0); // Bit fuer Anschlag A0 zuruecksetzen
            }
            else
            {
               //lcd_putc('a');
                cncstatus |= (1<< END_A0);      // Bit fuer Anschlag A0 setzen
                cncstatus &= ~(1<< COUNT_A);    // Motor A als relevant zuruecksetzen
                cncstatus |= (1<< COUNT_B);     // Motor B als relevant setzen
            }
        }
      
      // Anschlag B0     
      if ((CMD_PIN & (1<< END_B0_PIN)) ) // Schlitten nicht am Anschlag A1
      {
         cncstatus &= ~(1<< END_B0); // Bit fuer Anschlag A1 zuruecksetzen
      }
      else                            // Schlitten am Anschlag A1
      {
         if (richtung & (1<<RICHTUNG_B)) // Richtung ist auf Anschlag A1 zu, 
         {
            cncstatus |= (1<< END_B0);      // Bit fuer Anschlag B0 setzen
            cncstatus &= ~(1<< COUNT_B);    // Motor B als relevant zuruecksetzen
            cncstatus |= (1<< COUNT_A);     // Motor A als relevant setzen

            //lcd_putc(' ');
         }
         else // Richtung ist von  Anschlag A1 weg, nichts tun
         {
            cncstatus &= ~(1<< END_B0); // Bit fuer Anschlag A1 zuruecksetzen
           //lcd_putc('a');
         }
      }
      // End Anschlag B0
      */
      
      // Begin Motor A
      // **************************************
      // * Motor A *
      // **************************************
   
      // Es hat noch Steps, CounterA ist abgezaehlt (DelayA bestimmt Impulsabstand fuer Steps)
            if (StepCounterA &&(CounterA == 0) &&(!((cncstatus & (1<< END_A0))||(cncstatus & (1<< END_B0)))))//	
            {
                cli();
               // Impuls starten
               STEPPERPORT_1 &= ~(1<<MA_STEP);   // Impuls an Motor A LO -> ON
               CounterA=DelayA;                     // CounterA zuruecksetzen fuer neuen Impuls
               
               
               StepCounterA--;
               
               // Wenn StepCounterA abgelaufen und relevant: next Datenpaket abrufen
               if (StepCounterA ==0 && (cncstatus & (1<< COUNT_A)))    // Motor A ist relevant fuer Stepcount 
               {
                  
                  STEPPERPORT_1 |= (1<<MA_EN);                          // Motor A OFF
                  //STEPPERPORT_1 |= (1<<MB_EN);
                  StepCounterB=0; 
                  
                  // Begin Ringbuffer-Stuff
                  //if (ringbufferstatus & (1<<ENDBIT))
                  if (abschnittnummer==endposition)
                  {  
                     cli();
                      
                     ringbufferstatus = 0;
                     cncstatus=0;
                     sendbuffer[0]=0xAD;
                     sendbuffer[5]=abschnittnummer;
                     sendbuffer[6]=ladeposition;
                     usb_rawhid_send((void*)sendbuffer, 50);
                     sendbuffer[0]=0x00;
                     sendbuffer[5]=0x00;
                     sendbuffer[6]=0x00;
                     //AbschnittCounter=0;
                     //lcd_gotoxy(19,1);
                     //lcd_putc('!');
                     //lcd_putint2(abschnittnummer);
                     //lcd_putint2(ladeposition);
                     ladeposition=0;
                     sei();
                  }
                  else 
                  { 
                     uint8_t aktuellelage=0; // Lage innerhalb der Abschnittserie: Start: 1, Innerhalb: 0, Ende: 2
                     {
                        uint8_t aktuelleladeposition=(ladeposition & 0x00FF);
                        aktuelleladeposition &= 0x03;
                        // aktuellen Abschnitt laden
                        //lcd_putc('+');
                        //lcd_putint1(abschnittnummer);
                        if (ladeposition>8)
                        {
                           //lcd_putint1(ladeposition);
                        }
                        aktuellelage = AbschnittLaden(CNCDaten[aktuelleladeposition]);
                        if (aktuellelage==2) // war letzter Abschnitt
                        {
                           //lcd_gotoxy(14,1);
                           //lcd_putc('^');
                           endposition=abschnittnummer; // letzter Abschnitt
                        }  
                        else
                        {
                           // neuen Abschnitt abrufen
                           sendbuffer[5]=abschnittnummer;
                           sendbuffer[6]=ladeposition;
                           sendbuffer[0]=0xA1;
                           usb_rawhid_send((void*)sendbuffer, 50);
                           sendbuffer[0]=0x00;
                           sendbuffer[5]=0x00;
                           sendbuffer[6]=0x00;
                           
                        }
                        
                        ladeposition++;
                        
                     }
                     
                     
                     if (aktuellelage==2)
                     {
                        //ringbufferstatus |= (1<<ENDBIT);
                     }
                      AbschnittCounter++;

                  }
                  
               }
               
               sei();
            }
            else
            {
                STEPPERPORT_1 |= (1<<MA_STEP);					// Impuls an Motor A HI -> OFF
                
                if (StepCounterA ==0)							// Keine Steps mehr fuer Motor A
                {
                    STEPPERPORT_1 |= (1<<MA_EN);                     // Motor A OFF
                }
            }
          
        
       /*     
            // Halt-Pin
        else if (!(richtung & (1<<RICHTUNG_A)))
        {
            DelayA = 0;
            StepCounterA = 0;
            STEPPERPORT_1 |= (1<<MA_STEP);     // StepCounterA beenden
            STEPPERPORT_1 |= (1<<MA_EN);
            CounterA=0;
            
        }
        */
      // **************************************
      // * Motor B *
      // **************************************
      
		if (StepCounterB && (CounterB == 0))
		{
         cli();
         //lcd_putc('B');
         
			STEPPERPORT_1 &= ~(1<<MB_STEP);					// Impuls an Motor B LO ON
			CounterB= DelayB;
			StepCounterB--;
         
			if (StepCounterB ==0 && (cncstatus & (1<< COUNT_B))) // Motor B ist relevant fuer Stepcount 
			{
				STEPPERPORT_1 |= (1<<MB_EN);					// Motor B OFF
				
            StepCounterA=0;
            //lcd_putc('-');
            // Begin Ringbuffer-Stuff
            if (abschnittnummer==endposition)
            {  
               cli();
               
               ringbufferstatus = 0;
               cncstatus=0;
               sendbuffer[0]=0xAD;
               sendbuffer[1]=abschnittnummer;
               usb_rawhid_send((void*)sendbuffer, 50);
               sendbuffer[0]=0x00;
               sendbuffer[5]=0x00;
               sendbuffer[6]=0x00;

               //AbschnittCounter=0;
               //lcd_gotoxy(19,1);
               //lcd_putc('!');
               //lcd_putint1(abschnittnummer);
               //lcd_putint1(ladeposition);
               ladeposition=0;
               sei();
            }
            else 
            { 
               uint8_t aktuellelage=0;
               {
                  uint8_t aktuelleladeposition=(ladeposition & 0x00FF);
                  aktuelleladeposition &= 0x03;
                  // aktuellen Abschnitt laden
                  //lcd_putc('+');
                  //lcd_putint1(abschnittnummer);
 
                  aktuellelage = AbschnittLaden(CNCDaten[aktuelleladeposition]);
                  if (aktuellelage==2) // war letzter Abschnitt
                  {
                     //lcd_gotoxy(14,1);
                     //lcd_putc('^');
                     endposition=abschnittnummer; // letzter Abschnitt
                  }  
                  else
                  {
                     // neuen Abschnitt abruffen
                     sendbuffer[5]=abschnittnummer;
                     sendbuffer[6]=ladeposition;
                     sendbuffer[0]=0xA1;
                     usb_rawhid_send((void*)sendbuffer, 50);
                     sendbuffer[0]=0x00;
                     sendbuffer[5]=0x00;
                     sendbuffer[6]=0x00;

                  }
                  
                  ladeposition++;
                  
               }
               if (aktuellelage==2)
               {
                  //ringbufferstatus |= (1<<ENDBIT);
               }
               AbschnittCounter++;
               
            }
         }
			
			
			sei();
		}
		else// if (CounterB)
		{
			STEPPERPORT_1 |= (1<<MB_STEP);
			if (StepCounterB ==0)							// Keine Steps mehr fuer Motor B
			{
				STEPPERPORT_1 |= (1<<MB_EN);					// Motor B OFF
                
			}
			
			
			
		}
		sei(); 
      
      // End Motor B
      
      // Begin Motor C
      
      // **************************************
      // * Motor C *
      // **************************************
      
      // Es hat noch Steps, CounterC ist abgezaehlt (DelayA bestimmt Impulsabstand fuer Steps)
      if (StepCounterC &&(CounterC == 0) &&(!((cncstatus & (1<< END_C0))||(cncstatus & (1<< END_D0)))))//	
      {
         cli();
         // Impuls starten
         STEPPERPORT_2 &= ~(1<<MC_STEP);   // Impuls an Motor C LO -> ON
         CounterC=DelayC;                     // CounterA zuruecksetzen fuer neuen Impuls
         
         
         StepCounterC--;
         
         // Wenn StepCounterC abgelaufen und relevant: next Datenpaket abrufen
         if (StepCounterC ==0 && (cncstatus & (1<< COUNT_C)))    // Motor A ist relevant fuer Stepcount 
         {
            
            STEPPERPORT_2 |= (1<<MC_EN);                          // Motor C OFF
            StepCounterD=0; 
            
            // Begin Ringbuffer-Stuff
            //if (ringbufferstatus & (1<<ENDBIT))
            if (abschnittnummer==endposition)
            {  
               cli();
               
               ringbufferstatus = 0;
               cncstatus=0;
               sendbuffer[0]=0xAD;
               sendbuffer[5]=abschnittnummer;
               sendbuffer[6]=ladeposition;
               usb_rawhid_send((void*)sendbuffer, 50);
               sendbuffer[0]=0x00;
               sendbuffer[5]=0x00;
               sendbuffer[6]=0x00;
               //AbschnittCounter=0;
               //lcd_gotoxy(19,1);
               //lcd_putc('!');
               //lcd_putint2(abschnittnummer);
               //lcd_putint2(ladeposition);
               ladeposition=0;
               sei();
            }
            else 
            { 
               uint8_t aktuellelage=0; // Lage innerhalb der Abschnittserie: Start: 1, Innerhalb: 0, Ende: 2
               {
                  uint8_t aktuelleladeposition=(ladeposition & 0x00FF);
                  aktuelleladeposition &= 0x03;
                  // aktuellen Abschnitt laden
                  //lcd_putc('+');
                  //lcd_putint1(abschnittnummer);
                  if (ladeposition>8)
                  {
                     //lcd_putint1(ladeposition);
                  }
                  aktuellelage = AbschnittLaden(CNCDaten[aktuelleladeposition]);
                  if (aktuellelage==2) // war letzter Abschnitt
                  {
                     //lcd_gotoxy(14,1);
                     //lcd_putc('^');
                     endposition=abschnittnummer; // letzter Abschnitt
                  }  
                  else
                  {
                     // neuen Abschnitt abrufen
                     sendbuffer[5]=abschnittnummer;
                     sendbuffer[6]=ladeposition;
                     sendbuffer[0]=0xA2;
                     usb_rawhid_send((void*)sendbuffer, 50);
                     sendbuffer[0]=0x00;
                     sendbuffer[5]=0x00;
                     sendbuffer[6]=0x00;
                     
                  }
                  
                  ladeposition++;
                  
               }
               
               
               if (aktuellelage==2)
               {
                  //ringbufferstatus |= (1<<ENDBIT);
               }
               AbschnittCounter++;
               
            }
            
         }
         
         sei();
      }
      else
      {
         STEPPERPORT_2 |= (1<<MC_STEP);					// Impuls an Motor C HI -> OFF
         
         if (StepCounterC ==0)							// Keine Steps mehr fuer Motor C
         {
            STEPPERPORT_2 |= (1<<MC_EN);                     // Motor C OFF
         }
      }
      
        // **************************************
      // * Motor D *
      // **************************************
      
		if (StepCounterD && (CounterD == 0))
		{
         cli();
         //lcd_putc('D');
         
			STEPPERPORT_2 &= ~(1<<MD_STEP);					// Impuls an Motor D LO: ON
			CounterD= DelayD;
			StepCounterD--;
         
			if (StepCounterD ==0 && (cncstatus & (1<< COUNT_D))) // Motor D ist relevant fuer Stepcount 
			{
				STEPPERPORT_2 |= (1<<MD_EN);					// Motor D OFF
				
            StepCounterC=0;
            //lcd_putc('-');
            // Begin Ringbuffer-Stuff
            if (abschnittnummer==endposition)
            {  
               cli();
               
               ringbufferstatus = 0;
               cncstatus=0;
               sendbuffer[0]=0xAD;
               sendbuffer[1]=abschnittnummer;
               usb_rawhid_send((void*)sendbuffer, 50);
               sendbuffer[0]=0x00;
               sendbuffer[5]=0x00;
               sendbuffer[6]=0x00;
               
               //AbschnittCounter=0;
               //lcd_gotoxy(19,1);
               //lcd_putc('!');
               //lcd_putint1(abschnittnummer);
               //lcd_putint1(ladeposition);
               ladeposition=0;
               sei();
            }
            else 
            { 
               uint8_t aktuellelage=0;
               {
                  uint8_t aktuelleladeposition=(ladeposition & 0x00FF);
                  aktuelleladeposition &= 0x03;
                  // aktuellen Abschnitt laden
                  //lcd_putc('+');
                  //lcd_putint1(abschnittnummer);
                  
                  aktuellelage = AbschnittLaden(CNCDaten[aktuelleladeposition]);
                  if (aktuellelage==2) // war letzter Abschnitt
                  {
                     //lcd_gotoxy(14,1);
                     //lcd_putc('^');
                     endposition=abschnittnummer; // letzter Abschnitt
                  }  
                  else
                  {
                     // neuen Abschnitt abruffen
                     sendbuffer[5]=abschnittnummer;
                     sendbuffer[6]=ladeposition;
                     sendbuffer[0]=0xA3;
                     usb_rawhid_send((void*)sendbuffer, 50);
                     sendbuffer[0]=0x00;
                     sendbuffer[5]=0x00;
                     sendbuffer[6]=0x00;
                     
                  }
                  
                  ladeposition++;
                  
               }
               if (aktuellelage==2)
               {
                  //ringbufferstatus |= (1<<ENDBIT);
               }
               AbschnittCounter++;
               
            }
         }
			
			
			sei();
		}
		else// if (CounterB)
		{
			STEPPERPORT_2 |= (1<<MD_STEP);
			if (StepCounterD ==0)							// Keine Steps mehr fuer Motor D
			{
				STEPPERPORT_2 |= (1<<MD_EN);					// Motor D OFF
            
			}
			
			
			
		}
		sei(); 
      // End Motor D
		
   
		/**	Ende CNC-routinen	***********************/
		
		
		/* **** rx_buffer abfragen **************** */
		//rxdata=0;
		
#pragma mark rxdata		
		//	Daten von USB vorhanden
		 // rxdata
		
		//lcd_gotoxy(16,0);
        //lcd_putint(StepCounterA & 0x00FF);
		
		if (!(TASTENPIN & (1<<TASTE0))) // Taste 0
		{
			//lcd_gotoxy(8,1);
			//lcd_puts("T0 Down\0");
			
			if (!(TastenStatus & (1<<TASTE0))) //Taste 0 war noch nicht gedrueckt
			{
				//RingD2(5);
				TastenStatus |= (1<<TASTE0);
				
				Tastencount=0;
				//lcd_gotoxy(0,1);
				//lcd_puts("P0 \0");
				//lcd_putint(TastenStatus);
				//delay_ms(800);
			}
			else
			{
				
				
				Tastencount +=1;
				//lcd_gotoxy(7,1);
				//lcd_puts("TC \0");
				//lcd_putint(Tastencount);
				
				if (Tastencount >= Tastenprellen)
				{
               
					Tastencount=0;
               if (TastenStatus & (1<<TASTE0))
               {
                  //sendbuffer[0]=loopcount1;
                  //sendbuffer[1]=0xAB;
                  //usbstatus |= (1<<USB_SEND);
                  //lcd_gotoxy(2,1);
                  //lcd_putc('1');

                  //usb_rawhid_send((void*)sendbuffer, 50);
               }
					TastenStatus &= ~(1<<TASTE0);
               //lcd_gotoxy(3,1);
               //lcd_puts("ON \0");
               //delay_ms(400);
               //lcd_gotoxy(3,1);
              // lcd_puts("  \0");
               //lcd_putint(TastenStatus);
               
               
 					cncstatus |= (1<<TASTE0); // cncstatus &= ~(1<<TASTE0); ??
				}
			}//else
			
		}	// Taste 0
		
         
		
		if (!(TASTENPIN & (1<<TASTE1))) // Taste 1
		{
			//lcd_gotoxy(12,1);
			//lcd_puts("T1 Down\0");
			
			if (! (TastenStatus & (1<<TASTE1))) //Taste 1 war nicht nicht gedrueckt
			{
				TastenStatus |= (1<<TASTE1);
				Tastencount=0;
				//lcd_gotoxy(3,1);
				//lcd_puts("P1 \0");
				//lcd_putint(Servoimpulsdauer);
				//delay_ms(800);
				
			}
			else
			{
				//lcd_gotoxy(3,1);
				//lcd_puts("       \0");
				
				Tastencount +=1;
				if (Tastencount >= Tastenprellen)
				{
					
					
					Tastencount=0;
					TastenStatus &= ~(1<<TASTE1);
					
				}
			}//	else
			
		} // Taste 1
		
		/* ******************** */
		//		initADC(TASTATURPIN);
		//		Tastenwert=(uint8_t)(readKanal(TASTATURPIN)>>2);
		
		Tastenwert=0;
		
		//lcd_gotoxy(3,1);
		//lcd_putint(Tastenwert);
   
		//OSZI_B_HI;
      if (usbstatus & (1<< USB_SEND))
      {
         //lcd_gotoxy(10,1);
         //lcd_puthex(AbschnittCounter);
         //sendbuffer[3]= AbschnittCounter;
         //usb_rawhid_send((void*)sendbuffer, 50);
         //sendbuffer[0]=0;
         //sendbuffer[5]=0;
         //sendbuffer[6]=0;
         //usbstatus &= ~(1<< USB_SEND);
         
      }

	}//while
   //free (sendbuffer);

// return 0;
}
