#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>

#define LED_PIN  PD2  // LEDs
#define SET_PIN  PD1  // Set button
#define MODE_PIN PD0 // Mode button


int main(void) {
    // Настройка пина светодиода как выхода
    DDRD |= (1 << LED_PIN);
    // Init buttons
    PORTD = 0x00;
    

    while (1) {
    	if (!(PIND & 0x02)) { //Не нажата ли кнопка SET
    		PORTD |= 0x07;  //Включить LED
    	}
    	if (!(PIND & 0x01)) { //Не нажата ли кнопка MODE
    		PORTD &= 0xFB;  //Выключить LED
    	}
        // Включение светодиода
        //PORTD |= (1 << LED_PIN);

        // Пауза в 1 секунду
        //_delay_ms(1000);

        // Выключение светодиода
        //PORTD &= ~(1 << LED_PIN);

        // Пауза в 1 секунду
        //_delay_ms(1000);
    }

    return 0;
}

