/*
 *    Filename: dac.c
 * Description: Programm für den Audio-DAC von Albert Frisch, diesmal in C
 *     License: GPLv3 or later
 *     Depends: global.h, io.h, lcd.c
 *
 *      Author: Copyright (C) Philipp Hörauf, Max Gaukler and others
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Dieses Programm ist Freie Software: Sie können es unter den Bedingungen
 *  der GNU General Public License, wie von der Free Software Foundation,
 *  Version 3 der Lizenz oder (nach Ihrer Option) jeder späteren
 *  veröffentlichten Version, weiterverbreiten und/oder modifizieren.
 *
 *  Dieses Programm wird in der Hoffnung, dass es nützlich sein wird, aber
 *  OHNE JEDE GEWÄHRLEISTUNG, bereitgestellt; sogar ohne die implizite
 *  Gewährleistung der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK.
 *  Siehe die GNU General Public License für weitere Details.
 *
 *  Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
 *  Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>.
 */

/*
 *  Allgemeine Hinweise zu diesem Programm:
 *
 *  Die Firmware ist vorzüglich für einen Atmel mega16 geschrieben und
 *  getestet. Andere pinkompatible Prozessoren gehen sicher auch, es muss
 *  ggf. etwas geändert werden.
 *  Solche Änderungen sollen mit #ifdef eingebaut werden, so dass die
 *  Software vielfältiger wird.
 *  Die verschiedenen Keywords (Modus) sorgen dafür, dass der Atmel
 *  unterschiedlich mit seinen IOs umgeht, hier folgt eine Erklärung der Modi:
 *  - FRISCH:
 *  	vollständige Kompatibilität mit der orginalen Firmware von Albert Frisch.
 *  	auch das LCD muss nach den Frisch-Plänen angeschlossen werden.
 *  	einzige Änderung: der DAC läuft mit 24bit statt 16bit.
 *  	keine Lautstärkeregelung, Speicherung des aktiven Kanals beim Abschalten
 *  	, Kanalwechsel mittels Taster
 *
 *  - MAX:
 *    fast so wie FRISCH
 *    aber: AVCC muss an 5V angeschlossen werden
 *    anstatt Taster 2 wird ein Poti zwischen 5V und GND angeschlossen, um die Lautstärke einzustellen (Lautstärkeregelung über den DAC)
 *    Kanalwechsel geht nur noch mit Taster 1
 *  
 *  - SCHUHMANN:
 *  	Sehr zusammengesparte Version ohne LCD, dafür mit drei LEDs für die verschiedenen
 *  	Inputs. Lautstärkeregelung des DAC aktiv und über ADC des Atmels mit
 *  	einem POTI regelbar.
 *  	Automatische Quellenwahl aktiv (TODO)
 *
 *  - VIELE_IOS:
 *  	Momentan noch Spielwiese für Mods und Versuche mit den Controllern.
 *  	LCD kann relativ groß werden, vorgesehen sind 2x20 Zeichen, aber auch
 *  	LCDs mit zwei Controllern können mühelos angeschlossen werden.
 *  	Display muss über ein 74VHC595 angeschlossen werden (Standart_ON).
 *  	Lautstärkeregelung, ggf. sogar mit Master und Balance, integration von
 *  	Relaisausgang für eine Einschaltverzögerung, usw. usw..... viel TODO
 *
 *  Das Standart-Relais an PA0 ist in allen MODI enthalten.
 */

#include "../../../../atmel/lib/0.1.3/global.h"
#include "../../../../atmel/lib/0.1.3/io/io.h"
#include <avr/interrupt.h>
#include <math.h>

// Hier wird der Modus festgelegt, in dem das Programm läuft.

#define MAX

/* ATMEL PIN DISCRIPTION & DEFINITION

** freier Port: PORTC (LCD0 bis LCD7)	---> Erweiterungen
** Relais:      PA0			---> wird gelassen
** freie ADCs:  PA3, PA4		---> z.B. für Lautstärke-Poti
** freie Pins:  PA5, PA6, PA7		---> Schieberegister für LCD (VIELE_IOS-Modus)
*/

// PCM1796 (keine Adresse, da SPI-Modus)
#define PCM_RST   PB0     // Reset
#define PCM_DO    PB3     // Data Out		(MDI) vom ATMEL aus gesehen
#define PCM_CLK   PB2     // Clock		(MC)
#define PCM_DI    PB1     // Data In		(MDO) vom ATMEL aus gesehen
#define PCM_CS    PB4     // Chip Select	(MS)

// AD1896
#define AD_MUTE PD0       // Mute
#define AD_RST  PD1       // Reset

// CS8416
#define CS_ADR 0b0010000  // Chip Adress (ja, das sind WIRKLICH 7 bit)
#define CS_PORT PORTD
#define CS_PIN PIND
#define CS_DO   PD4       // Data Out vom ATMEL aus gesehen
#define CS_CLK  PD3       // Clock
#define CS_DI   PD2       // Data In vom ATMEL aus gesehen
#define CS_RST  PD5       // Reset
#define CS_CS   PD6       // Chip Select


#if defined(FRISCH) || defined(MAX) // 4bit LCD, kompatibel zur Originalfirmware

#define LCD_MODE_4BIT
#define LCD_PORT PORTA
#define LCD_DDR DDRA
#define LCD_EN_PIN PA7
#define LCD_RW_PIN PA6
#define LCD_RS_PIN PA5
#define LCD_FREE_DATA_LINES
#define LCD_DATA_DDR DDRC
#define LCD_DATA_PORT PORTC
#define LCD_D4_PIN PC3
#define LCD_D5_PIN PC2
#define LCD_D6_PIN PC1
#define LCD_D7_PIN PC0
#define LCD_LINES 2
#define LCD_COLS 16
#include "../../../../atmel/lib/0.1.3/io/display/lcd/lcd.c"
#include "../../../../atmel/lib/0.1.3/io/entprellung.h"
#include "../../../../atmel/lib/0.1.3/global.h"
entprell_struct taster1, taster2;
#define TASTER1_EVENT() in_entprellt_rising(&taster1, PINA, PA4, 1)
#define TASTER2_EVENT() in_entprellt_rising(&taster2, PINA, PA3, 1)
#include <avr/eeprom.h>
EEMEM uint8_t storedChannel=0;
#endif

#ifdef SCHUHMANN
/*	kein Display, nur LEDs für:
	Eingang - USB
	Eingang - S/PDIF
	Eingang - HDMI
	Software - Mute
	Lautstärkeregler
*/
#define LED_DDR  DDRC
#define LED_PORT PORTC
#define OPTO PC0
#define HDMI PC1
#define USB  PC2
#define LED4 PC3

#endif

#ifdef VIELE_IOS
/* 	alternative Anschlussform
	LCD: Schieberegister
	möglichst viele freie IO-PORTs
	Sonderfunktionen!!
*/
#define LCD_SHIFT_PORT PORTA
#define LCD_SHIFT_DDR DDRA
#define LCD_SHIFT_DATA_PIN PA5
#define LCD_SHIFT_CLOCK_PIN PA6
#define LCD_SHIFT_LATCH_PIN PA7
#define LCD_EN_PIN 6
#define LCD_RS_PIN 5
#define LCD_D4_PIN 1
#define LCD_LINES 2
#define LCD_COLS 20
#define LCD_MODE_SHIFT_4BIT
#include "../../../atmel/lib/0.1.3/io/display/lcd/lcd.c"
#endif

void cs_clock(void) {
	sbi(PORTD, CS_CLK);
	delayus(2);
	cbi(PORTD, CS_CLK);
	delayus(1);
}

void spi_tx_cs(uint8_t reg, uint8_t wert) {
	cbi(CS_PORT, CS_CS);
	delayus(10);
	for (int8_t i=6; i>=0; i--) { // Sende Adresse
		if(CS_ADR & (1<<i)) {
			sbi(CS_PORT, CS_DO); // sende 1
		} else {
			cbi(CS_PORT, CS_DO); // sende 0
		}
		cs_clock();
	}
	
	cbi(CS_PORT, CS_DO); // sende 0 = Write
	
	cs_clock();
	for (int8_t i=7; i>=0; i--) { // Sende MAP (Pointer auf die richtige RAM-Adresse)
		if(reg & (1<<i)) {
			sbi(CS_PORT, CS_DO); // sende 1
		} else {
			cbi(CS_PORT, CS_DO); // sende 0
		}
		cs_clock();
	}
	for (int8_t i=7; i>=0; i--) { // Sende Daten
		if(wert & (1<<i)) {
			sbi(CS_PORT, CS_DO); // sende 1
		} else {
			cbi(CS_PORT, CS_DO); // sende 0
		}
		cs_clock();
	}
	sbi(CS_PORT, CS_CS);
	delayus(2);
}

uint8_t spi_rx_cs(uint8_t reg) {
	/// Register im CS8416 SPDIF RX auslesen
	
	// Adresse setzen: dazu ein Schreibzyklus ohne Daten
	
	cbi(CS_PORT, CS_CS);
	delayus(10);
	for (int8_t i=6; i>=0; i--) { // Sende Adresse
		if(CS_ADR & (1<<i)) {
			sbi(CS_PORT, CS_DO); // sende 1
		} else {
			cbi(CS_PORT, CS_DO); // sende 0
		}
		cs_clock();
	}
	
	cbi(CS_PORT, CS_DO); // sende 0 (TX)
	cs_clock();
	for (int8_t i=7; i>=0; i--) { // Sende MAP (Pointer auf die richtige RAM-Adresse)
		if(reg & (1<<i)) {
			sbi(CS_PORT, CS_DO); // sende 1
		} else {
			cbi(CS_PORT, CS_DO); // sende 0
		}
		cs_clock();
	}
	delayus(10);
	sbi(CS_PORT, CS_CS);
	delayus(10);
	
	// Daten lesen: Chip Address, RW, dann Daten einlesen
	
	cbi(CS_PORT, CS_CS);
	delayus(10);
	for (int8_t i=6; i>=0; i--) { // Sende Adresse
		if(CS_ADR & (1<<i)) {
			sbi(CS_PORT, CS_DO); // sende 1
		} else {
			cbi(CS_PORT, CS_DO); // sende 0
		}
		cs_clock();
	}
	sbi(CS_PORT, CS_DO); // sende 1 (RX)
	
	uint8_t read=0;
	for (int8_t i=7; i>=0; i--) { // Lese Daten
		cs_clock();
		delayus(2);
		if (CS_PIN & (1<<CS_DI)) {
			read |= (1<<i);
		}
	}
	
	delayus(10);
	sbi(CS_PORT, CS_CS);
	delayus(10);
	return read;
}

void pcm_clock(void) {
	delayus(1);
	sbi(PORTB, PCM_CLK);
	delayus(1);
	cbi(PORTB, PCM_CLK);
	delayus(1);
}

void spi_tx_pcm(uint8_t reg, uint8_t wert) { // keine Adresse, da im SPI-Mode
	cbi(PORTB, PCM_CS);
	delayus(10);
	for (int8_t i=7; i>=0; i--) { // Sende MAP (Pointer auf die richtige RAM-Adresse)
		if(reg & (1<<i)) {
			sbi(PORTB, PCM_DO); // sende 1
		} else {
			cbi(PORTB, PCM_DO); // sende 0
		}
		pcm_clock();
	}
	for (int8_t i=7; i>=0; i--) { // Sende Daten
		if(wert & (1<<i)) {
			sbi(PORTB, PCM_DO); // sende 1
		} else {
			cbi(PORTB, PCM_DO); // sende 0
		}
		pcm_clock();
	}
	sbi(PORTB, PCM_CS);
	delayus(2);
}

void UpsamplerInit(void) { // Initialisiere AD Upsampler - nicht viel zu tun, da Hardware-Mode!
	DDRD |= (1<<AD_MUTE) | (1<<AD_RST);
	sbi(PORTD, AD_RST);
	cbi(PORTD, AD_MUTE);
	delayms(1);
	sbi(PORTD, AD_MUTE);
}

void ReceiverInit(void) { // Initialisiere SPDIF Line Receiver - muss noch extra aktiviert und auf den richtigen Eingang gesetzt werden!!!
	DDRD |= (1<<CS_DO) | (1<<CS_CLK) | (1<<CS_RST) | (1<<CS_CS);
	delayms(1);
	sbi(PORTD, CS_CS);
	sbi(PORTD, CS_RST);		// Chip aktivieren
	delayus(1);
	cbi(PORTD, CS_CS);		// SPI Mode auswählen CS high-low after RST low-high
	delayus(1);
	sbi(PORTD, CS_CLK);
	sbi(PORTD, CS_CS);
	delayms(5);

	spi_tx_cs(0x00, 0b00001000);	// higher update ate Phase Detector for 32 to 108 kHz Sampling rate
	spi_tx_cs(0x01, 0b00000100);	// mute on error
	spi_tx_cs(0x02, 0b00001011);	// GPO0 gibt SPDIF Passthrough aus
	spi_tx_cs(0x03, 0b00000000);	// GPO1, GPO2 ausgeschaltet
	spi_tx_cs(0x05, 0b11000000);	// MasterMode, OSCLK=128fs, 24-bit Daten, left-justified ---> 1110 0000 war Voreinstellung vom Frisch
	spi_tx_cs(0x06, 0b11111111);	// Receiver Error Mask alles an, damit "mute on error" arbeiten kann.
}

void ReceiverSetChannel(uint8_t ch) {
	if (ch <= 2) {
		spi_tx_cs(0x04, ((1<<7) | (ch<<3) | ch)); // Setting Passthrough & SPIDF Input
	}
#ifdef SCHUHMANN
	LED_PORT &= ~((1<<OPTO) | (1<<HDMI) | (1<<USB)); // clear LEDs
	if (ch == 0) {
		sbi(LED_PORT, OPTO);
	} else if (ch == 1) {
		sbi(LED_PORT, HDMI);
	} else {
		sbi(LED_PORT, USB);
	}
#endif
}

uint16_t ReceiverGetSamplerate(void) {
	/// return the samplerate in units of 0.1kHz
	// Register 18h:
	// ratio=Fso/Fsi * 64
	uint8_t ratio=spi_rx_cs(0x18);
	
	// OMCK=256*Fso  = Quartz frequency
	// Fsi = input samplerate
	
	// solving these equations
	// => samplerate [Hz] = fQuartz / (4 * ratio)
	// => samplerate [0.1kHz] = samplerate / 100
	const uint32_t fQuartz=24576000;
	uint32_t samplerate=(fQuartz/(100*4))/ratio;
	if (samplerate > UINT16_MAX) {
		samplerate=UINT16_MAX;
	}
	return (uint16_t)samplerate;
}

void DacSetVolume(uint8_t left, uint8_t right) { // 255 == volle Lautstärke
	spi_tx_pcm(16, left);
	spi_tx_pcm(17, right);
	spi_tx_pcm(18, 0b10101000);
}

void DacInit(void) {
	DDRB |= (1<<PCM_RST) | (1<<PCM_DO) | (1<<PCM_CLK) | (1<<PCM_CS);
	sbi(PORTB, PCM_CS);
	sbi(PORTB, PCM_RST);

 	spi_tx_pcm(16, 0);		// Lautstärke auf minimum Links
 	spi_tx_pcm(17, 0);		// Lautstärke auf minimum Rechts ---> wird später gesetzt.
	spi_tx_pcm(18, 0b10101000);	// 24bit Standart Data, De-emphasis disabled, not muted, Lautstärkeregler an
 	spi_tx_pcm(19, 0b01000000);	// attenuation rate langsamer
	spi_tx_pcm(20, 0b00000010);	//
// #warning DEBUG DAC INIT
}

void adcStartConversion(void) {
// 	ADMUX &= ~((1<<MUX4) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (1<<MUX0));
// 	ADMUX |= channel;
	ADCSRA |= (1<<ADSC);
}

uint8_t adcReady (void) {
	if (ADCSRA & (1<<ADIF)) {
		return 1;
	} else {
		return 0;
	}
}

uint8_t zufall(uint8_t data, uint8_t seed) {
	uint8_t i, y=seed;
		y^=data;
		y=y<<1|(y>>7); //eins nach links rotieren
	return y; // noch ein paar bits hin und her kippen
}

#if defined(SCHUHMANN) || defined(VIELE_IOS)
volatile uint8_t seed = 0;
ISR(TIMER1_COMPA_vect,ISR_BLOCK) { // Interrupt-Routine
	static uint8_t zeit=0;
	seed = zufall(seed, seed);
	if (seed > 128) {
		sbi(LED_PORT, LED4);
	} else  {
		cbi(LED_PORT, LED4);
	}
// 	LED_PORT ^= (1<<LED4) | (1<<OPTO) | (1<<HDMI) | (1<<USB);
	zeit ++;
	if(zeit == 214) { // eine Stunde ist vergangen...
		zeit = 0;
	}
}
#endif

int main(void) {

	delayms(500); // Warten, damit alle Clocks stabil laufen UND das LCD bereit ist.
	DDRA |= (1<<PA0); // Relais-Ausgang
	uint8_t activeChannel = 2; // (0: TOSLINK --- 1: Coax-S/PDIF --- 2: USB)
	ReceiverInit();
	UpsamplerInit();
	DacInit();
	delayms(100);
	char tmpstr[20];
	#ifdef MAX
		#define ADC_AVERAGE_SAMPLES 30
		uint16_t adcSampleBuffer[ADC_AVERAGE_SAMPLES];
		uint8_t adcSampleBufferIndex=0;
	#endif
	
	
	#if defined(FRISCH) || defined(MAX)
		lcd_init();
		lcd_print("   Audio-DAC    ");
		lcd_set_position(16);
		lcd_putstr_P(PSTR("    starte     "));
		DacSetVolume(0,0); // erstmal Volume runterdrehen
		delayms(500);
		#if defined(FRISCH)
			PORTA |= (1<<PA3) | (1<<PA4);
		#else
			PORTA |= (1<<PA4); // Taster-Pullup - an PA3 hängt das Poti für Lautstärke, dort kein Pullup
			// ADC - Init
			ADMUX = (1<<REFS0) | (0<<ADLAR) |  (1<<MUX1) | (1<<MUX0); // 10 bit Abtastung, Kanal AD3
			ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1); // fadc = fcpu/64
			adcStartConversion();
			while (adcReady() == 0); // warte, bis ADC-auslesung fertig ist
			adcStartConversion();
			// initialisiere running-average Filter
			for (uint8_t i=0; i<ADC_AVERAGE_SAMPLES; i++) {
				adcSampleBuffer[i]=0;
			}
		#endif
	#endif
		
		
	#ifdef SCHUHMANN
		LED_DDR |= (1<<OPTO) | (1<<HDMI) | (1<<USB) | (1<<LED4);

		// TIMER init
		// F_CPU = 4MHz
		TCCR1B = (1<<WGM12) | (1<<CS12) | (0<<CS11) | (1<<CS10); // CTC mode - TOP = OCR1A, prescaler == 1024
		TIMSK = (1<<OCIE1A); // overflow interrupt enabled
		OCR1A = 1000; // full 16bit
	#endif

	#ifdef VIELE_IOS
		lcd_init();
		lcd_print("true 24bit Audio DAC");
		lcd_set_position(20);
		lcd_putstr(" Input: USB PCM2704 "); // Standart-input, da dieser DAC erst mal am PC genutzt wird.
	#endif

	ReceiverSetChannel(activeChannel);
	cbi(PORTD, AD_MUTE); // Software-Mute aus
	sbi(PORTA, PA0);     // Ausgangsrelais AKTIVIEREN

	#if defined(FRISCH) || defined(MAX)
	delayms(100);
	DacSetVolume(255,255); // nach dem Anschalten des Relais langsam die Lautstärke hochfahren
	
	activeChannel=eeprom_read_byte(&storedChannel);
	uint8_t eeprom_store_delay=0; // wird auf 255 gesetzt, um nach Änderungen zeitverzögert den Channel ins ROM zu speichern
	#ifdef MAX
		uint32_t volume=0;
		uint32_t lastVolume=0;
		uint8_t lastRealVolume=0;
		const uint8_t volumeHysteresis=2*ADC_AVERAGE_SAMPLES; // 0.5 volume-units hysteresis
	#endif
	while(1) {
		if (TASTER1_EVENT()) {
			activeChannel++;
			if (activeChannel>2) {
				activeChannel=0;
			}
			eeprom_store_delay=255;
		}
		#ifdef FRISCH
			if (TASTER2_EVENT()) {
				if (activeChannel==0) {
					activeChannel=2;
				} else {
					activeChannel--;
				}
				eeprom_store_delay=255;
			}
		#endif
		if (eeprom_store_delay>0) {
			eeprom_store_delay--;
			if (eeprom_store_delay==1) {
				eeprom_update_byte(&storedChannel,activeChannel);
			}
		}
		delayms(1);
		lcd_set_position(0);
		ReceiverSetChannel(activeChannel);
		#ifdef FRISCH
			if (activeChannel==0) {
				lcd_putstr_P(PSTR("In: Opto SPDIF  "));
			} else if (activeChannel==1) {
				lcd_putstr_P(PSTR("In: Koax SPDIF  "));
			} else if (activeChannel==2) {
				lcd_putstr_P(PSTR("In: USB PCM2704 "));
			} else {
				lcd_putstr_P(PSTR("Fehler"));
			}
		#endif
		
		#ifdef MAX
			// Display line 1:
			// Opto SPDIF 44.1k
			if (activeChannel==0) {
				lcd_putstr_P(PSTR("Opto SPDIF "));
			} else if (activeChannel==1) {
				lcd_putstr_P(PSTR("Koax SPDIF "));
			} else if (activeChannel==2) {
				lcd_putstr_P(PSTR("USB        "));
			} else {
				lcd_putstr_P(PSTR("Fehler"));
			}
			uint8_t rxError=spi_rx_cs(0x0C);
			// TODO Sampleraten-Erkennung geht nicht.
			lcd_putstr_P(PSTR("     "));
// 			if (rxError) {
// 				lcd_putstr_P(PSTR("--.-"));
// 			} else {
// 				uint16_t samplerate=ReceiverGetSamplerate();
// 				//snprintf(tmpstr,sizeof(tmpstr),"%2u.%01u",samplerate/10,samplerate%10);
// 				snprintf(tmpstr,sizeof(tmpstr),"%u.%01u",samplerate/10,samplerate%10);
// 				lcd_putstr(tmpstr);
// 			}
// 			lcd_putchar('k');
			if(adcReady()) {
				adcSampleBuffer[adcSampleBufferIndex]=ADC;
				adcStartConversion();
				adcSampleBufferIndex++;
				if (adcSampleBufferIndex >= ADC_AVERAGE_SAMPLES) {
					adcSampleBufferIndex=0;
				}
				volume=0;
				for (uint8_t i=0; i<ADC_AVERAGE_SAMPLES; i++) {
					volume += adcSampleBuffer[i];
				}
				
				
			}
			// Hysteresis so that small ADC noise does not cause the volume to toggle between n and n+1:
			// only change volume if the value has changed enough
			if ((volume > lastVolume + volumeHysteresis) || (volume + volumeHysteresis < lastVolume)) {
				uint16_t realVolume=volume/(4*ADC_AVERAGE_SAMPLES);
				DacSetVolume(realVolume,realVolume);
				lastVolume=volume;
				lastRealVolume=realVolume;
			}
			if (rxError) {
				// Fehler
				lcd_putstr_P(PSTR(" kein Signal    "));
			} else {
				// Vol -123.5 dBFS
				lcd_putstr_P(PSTR("Vol "));
// 				snprintf(tmpstr,sizeof(tmpstr),"%03d",volume);
				
				snprintf(tmpstr,sizeof(tmpstr),"%4d",-(int16_t)lastRealVolume/2);
				lcd_putstr(tmpstr);
				lcd_putchar('.');
				if (lastRealVolume%2) {
					lcd_putchar('5');
				} else {
					lcd_putchar('0');
				}
				lcd_putstr_P(PSTR("dBFS"));
			}
			
		#endif
		#ifdef FRISCH
			DacSetVolume(255, 255);
			if (spi_rx_cs(0x0C) != 0) {
				// Fehler
				lcd_putstr_P(PSTR(" kein Signal    "));
			} else {
				lcd_putstr_P(PSTR("      Signal    "));
			}
		#endif
	}
	#endif

	#ifdef SCHUHMANN
	uint16_t adcTemp = 0;
	uint8_t i = 0, j = 0, poti = 0, potiOld = 0;
	LED_PORT |= (1<<OPTO) | (1<<HDMI) | (1<<USB) | (1<<LED4);
// 	sei(); // interrupts Aktiv!!!
	while(1) {
		if(i == 4) {
			i = 0;
			potiOld = poti;
			poti = (adcTemp / 51);
			adcTemp = 0;
		}
		if(adcReady()) {
			adcTemp += ADC;
			i++;
			j++;
			adcStartConversion();
		}
		if (potiOld != poti) { // Lautstärke hat sich geändert ---> neu setzen
			DacSetVolume(poti+155, poti+155);
		}
	}
	#endif

	return 0;
}
