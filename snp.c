#define F_CPU 1000000UL

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <stdbool.h>
#include <stdint.h>
#include <util/delay.h>

#define SECS_PER_MIN (uint32_t)60
#define SECS_PER_HOUR ((uint32_t)60 * SECS_PER_MIN)
#define SECS_PER_DAY ((uint32_t)24 * SECS_PER_HOUR)

volatile uint32_t millis = 0;
volatile uint32_t last_activity = 0;
volatile uint32_t secs = 0;

volatile enum Mode { Display, Sleep, SetTime, Measure } mode = Display;

volatile enum Intensity {
    VeryLow,
    Low,
    Medium,
    High,
    VeryHigh,
} intensity = Medium;

void apply_intensity() {
    const uint8_t mapping[5] = {32, 64, 128, 192, 255};
    uint8_t inverted = 255 - mapping[(uint8_t)intensity];
    OCR1A = inverted;
    OCR1B = inverted;
}

void increase_intensity() {
    if (intensity == VeryHigh) {
        intensity = VeryLow;
    } else {
        intensity += 1;
    }
    apply_intensity(intensity);
}

void decrease_intensity() {
    if (intensity == VeryLow) {
        intensity = VeryHigh;
    } else {
        intensity -= 1;
    }
    apply_intensity(intensity);
}

void setup_timer0_millis() {
    // prescaler 8
    TCCR0B |= (1 << CS01);

    // enable ctc
    TCCR0A |= (1 << WGM01);

    // enable Timer0 compare interrupt A
    TIMSK0 |= (1 << OCIE0A);
    OCR0A = 125 - 1;
}

ISR(TIMER0_COMPA_vect) { millis += 1; }

void setup_timer1_pwm() {
    // fast PWM 8-bit
    TCCR1A |= (1 << WGM10);
    TCCR1B |= (1 << WGM12);

    TCCR1A |= (1 << COM1A1) | (1 << COM1B1);

    // prescaler 8
    TCCR1B |= 1 << CS11;

    // set OC1A, OC1B writeable
    DDRB |= (1 << PB1) | (1 << PB2);
}

void setup_timer2_secs() {
    // prescaler 128
    TCCR2B |= (1 << CS22) | (1 << CS20);

    // Asynchronous Counter2, with frequency 2^15
    ASSR |= (1 << AS2);

    // Run interrupt on Counter2 overflow
    TIMSK2 |= (1 << TOIE2);
    TIFR2 |= (1 << TOV2);
}

ISR(TIMER2_OVF_vect) { secs += 1; }

const uint8_t mask_c =
    (1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC4) | (1 << PC5);
const uint8_t mask_d = (1 << PD0) | (1 << PD1) | (1 << PD6) | (1 << PD7);
const uint8_t mask_b = (1 << PB0);

void setup_leds() {
    apply_intensity(128);
    DDRC |= mask_c;
    DDRD |= mask_d;
    DDRB |= mask_b;
}

uint16_t extract_bit(uint16_t bits, uint8_t index) {
    return (bits >> index) & 0b1;
}

void display(uint16_t bits) {
    PORTC &= ~mask_c;
    PORTD &= ~mask_d;
    PORTB &= ~mask_b;

    PORTC |= (extract_bit(bits, 0) << PC0) | (extract_bit(bits, 1) << PC1) |
             (extract_bit(bits, 2) << PC2) | (extract_bit(bits, 3) << PC3) |
             (extract_bit(bits, 4) << PC4) | (extract_bit(bits, 5) << PC5);
    PORTD |= (extract_bit(bits, 6) << PD0) | (extract_bit(bits, 7) << PD1) |
             (extract_bit(bits, 8) << PD6) | (extract_bit(bits, 9) << PD7);
    PORTB |= (extract_bit(bits, 10) << PB0);
}

void invert_display() {
    PORTC ^= mask_c;
    PORTD ^= mask_d;
    PORTB ^= mask_b;
}

void setup_buttons() {
    // enable external interrupts INT0 and INT1
    EIMSK |= (1 << INT0) | (1 << INT1);

    // trigger interrupt on a falling edge
    EICRA |= (1 << ISC01) | (1 << ISC11);

    // enable pin change interrupts PCINT23..16
    PCICR |= (1 << PCIE2);

    // enable pin change interrupt PCINT20
    PCMSK2 |= (1 << PCINT20);

    // enable internal pullup resistors
    PORTD |= (1 << PD2) | (1 << PD3) | (1 << PD4);
}

int32_t difference(uint32_t a, uint32_t b) {
    return (a > b) ? a - b : -(b - a);
}

bool debounce(uint32_t delay) {
    bool result = difference(millis, last_activity) > delay;
    if (result) {
        last_activity = millis;
    }
    return result;
}

ISR(INT0_vect) {
    if (!debounce(250)) return;

    if (mode == Display) {
        mode = SetTime;
        invert_display();
        return;
    }
    if (mode == SetTime) {
        mode = Display;
        invert_display();
        return;
    }
    if (mode == Measure) {
        mode = Display;
        return;
    }
    if (mode == Sleep) {
        sleep_disable();
        mode = Display;
        return;
    }
}

ISR(INT1_vect) {
    if (mode == Sleep) {
        sleep_disable();
        apply_intensity(255);
        mode = Measure;
        return;
    }
    if (!debounce(250)) return;

    if (mode == Display) {
        decrease_intensity();
        return;
    }

    secs += SECS_PER_HOUR;
    if (secs >= SECS_PER_DAY) {
        secs -= SECS_PER_DAY;
    }
}

ISR(PCINT2_vect) {
    if (mode == Sleep) return;
    if (!debounce(250)) return;

    if (mode == Display) {
        increase_intensity();
        return;
    }

    secs += SECS_PER_MIN;
    if (secs % SECS_PER_HOUR == 0) {
        secs -= SECS_PER_HOUR;
    }
}

int main() {
    setup_timer0_millis();
    setup_timer1_pwm();
    setup_timer2_secs();
    setup_leds();
    setup_buttons();
    set_sleep_mode(SLEEP_MODE_PWR_SAVE);
    sei();

    while (true) {
        if (mode == Sleep) {
            sleep_enable();
            continue;
        }

        if (mode == Measure) {
            if (secs % 2 == 0)
                display(0b11111111111);
            else
                display(0);
            continue;
        }

        // sleep after 10 seconds of inactivity
        if (difference(millis, last_activity) >= 10000) {
            display(0);
            mode = Sleep;
            continue;
        }

        // hours in the upper 5 bits
        const uint16_t hours = secs / SECS_PER_HOUR;
        // minutes in the lower 6 bits
        const uint16_t mins = (secs % SECS_PER_HOUR) / SECS_PER_MIN;

        display(hours << 6 | mins);

        _delay_ms(250);
    }
}
