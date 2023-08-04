#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define LED_PIN  PD2  // LEDs
#define SET_PIN  PD1  // Set button
#define MODE_PIN PD0  // Mode button
#define NEON_PIN PD5  // Neon light

void timer1_init() {
    //Set timer 1 to CTC mode (Clear Timer on Compare
    TCCR1B = (1 << WGM12);
    
    // Установка предделителя таймера 1 в 256 (
    TCCR1B |= (1 << CS12);

    // Установка начального значения счетчика таймера 0 (для 1 секунды при F_CPU=8МГц)
    //TCNT1 = 65536 - 60000; // 256 - (8000000 / 1024 / 2)

    //OCR - Output Compare Register
    OCR1A = 31250;

    // Разрешение прерывания по переполнению таймера 0
    TIMSK = (1 << OCIE1A);
}

int extraTime = 0;

int main(void) {
    // Настройка пина светодиода как выхода
    DDRD |= (1 << LED_PIN);
    DDRD |= (1 << NEON_PIN);
    // Init buttons
    PORTD = 0x00;
    timer1_init();
    sei();
    

    while (1) {
    	/*if (!(PIND & 0x02)) { //Не нажата ли кнопка SET
    		PORTD |= 0x07;  //Включить LED
    	}
    	if (!(PIND & 0x01)) { //Не нажата ли кнопка MODE
    		PORTD &= 0xFB;  //Выключить LED
    	} */
        // Включение светодиода
        PORTD |= (1 << NEON_PIN);

        // Пауза в 1 секунду
        _delay_ms(1000);

        // Выключение светодиода
        PORTD &= ~(1 << NEON_PIN);

        // Пауза в 1 секунду
        _delay_ms(1000);
    }

    return 0;
}

ISR(TIMER1_COMPA_vect) {
    //extraTime++;
    
    //if (extraTime > 100) {
      PORTD^= (1 << LED_PIN);
    //}
    
}
