---
title: Systemnahe Programmierung Binäruhr
author: Nika Sommer
---

Funktionen
---

# Zeit einstellen

- Mode switch button drücken
- Stunden und Minuten mit beiden anderen einstellen

# Dimmen

- Mode switch button drücken
- dimmer mit beiden anderen buttons

# Sleep Mode

- 10 Sekunden warten

# Uhrzeit läuft weiter

- nach ca. 1 Minute wieder aufwachen lassen

<!-- end_slide -->

Button Interrupts
---

```c
// enable external interrupts INT0 and INT1
EIMSK |= (1<<INT0) | (1<<INT1);

// trigger interrupt on a falling edge
EICRA |= (1<<ISC01) | (1<<ISC11);

// enable pin change interrupts PCINT23..16
PCICR |= (1<<PCIE2);

// enable pin change interrupt PCINT20
PCMSK2 |= (1<<PCINT20);

// enable internal pullup resistors
PORTD |= (1<<PD2) | (1<<PD3) | (1<<PD4);
```

<!-- end_slide -->

INT0
---

<!-- column_layout: [1, 1] -->

<!-- column: 0 -->

- Mode Select & Wakeup

<!-- column: 1 -->


```c
ISR(INT0_vect) {
    if (!debounce(250)) return;

    switch (mode) {
        case Display:
            mode = SetTime;
            invert_display();
            break;
        case SetTime:
            mode = Display;
            invert_display();
            break;
        case Sleep:
            sleep_disable();
            mode = Display;
            break;
    }
}
```

<!-- end_slide -->

INT1
---

<!-- column_layout: [1, 1] -->

<!-- column: 0 -->

- Sleep: ---
- Display: Helligkeit verringern
- SetTime: Stunden +1

<!-- column: 1 -->

```c
ISR(INT1_vect) {
    if (mode == Sleep) return;
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
```

<!-- end_slide -->

PCINT20
---

<!-- column_layout: [1, 1] -->

<!-- column: 0 -->

- Sleep: ---
- Display: Helligkeit erhöhen
- SetTime: Minuten +1

<!-- column: 1 -->

```c
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
```

<!-- end_slide -->

Timer Interrupts
---

# Timer0, Milliseconds

<!-- column_layout: [1, 1] -->

<!-- column: 0 -->

```
Frequenz = 1MHz
Prescaler = 8
Periode = 1ms

Periode = (Prescaler * (Compare + 1))
          / Frequenz
Compare = (Periode * Frequenz)
          / Prescaler - 1
Compare = (1ms * 1Mhz) / 8 - 1
        = 125 - 1
```

<!-- column: 1 -->

```c
void setup_timer0_millis() {
    // prescaler 8
    TCCR0B |= (1 << CS01);

    // enable ctc
    TCCR0A |= (1 << WGM01);

    // enable Timer0 compare interrupt A
    TIMSK0 |= (1 << OCIE0A);
    OCR0A = 125 - 1;
}
```

<!-- end_slide -->

Timer Interrupts
---

# Timer1, PWM

<!-- column_layout: [1, 1] -->

<!-- column: 0 -->

```c
void setup_timer1_pwm() {
    // fast PWM 8-bit
    TCCR1A |= (1 << WGM10);
    TCCR1B |= (1 << WGM12);

    // Use OC1A and OC1B pins
    TCCR1A |= (1 << COM1A1) | (1 << COM1B1);

    // prescaler 8
    TCCR1B |= 1 << CS11;

    // set OC1A, OC1B writeable
    DDRB |= (1 << PB1) | (1 << PB2);
}
```

<!-- column: 1 -->

```c
volatile enum Intensity {
    VeryLow = 0,
    Low = 1,
    Medium = 2,
    High = 3,
    VeryHigh = 5,
} intensity = Medium;

void apply_intensity(
    enum Intensity intensity
) {
    const uint8_t mapping[5]
        = {32, 64, 128, 192, 255};
    uint8_t inverted
        = 255 - mapping[(uint8_t)intensity];
    OCR1A = inverted;
    OCR1B = inverted;
}
```

<!-- end_slide -->

Timer Interrupts
---

# Timer2, Seconds

<!-- column_layout: [1, 1] -->

<!-- column: 0 -->

```
Frequenz = 2^15Hz
Width = 256
Prescaler = 128

Periode = (Prescaler * Width) / Frequenz
Periode = (128 * 256) / 2^15Hz
        = 1s
```

<!-- column: 1 -->

```c
void setup_timer2_secs() {
    // prescaler 128
    TCCR2B |= (1 << CS22) | (1 << CS20);

    // Asynchronous Counter2,
    // with frequency 2^15
    ASSR |= (1 << AS2);

    // Run interrupt on Counter2 overflow
    TIMSK2 |= (1 << TOIE2);
    TIFR2 |= (1 << TOV2);

    // measure output pin
    DDRD |= (1 << PD5);
}
```

<!-- end_slide -->

Genauigkeitsmessung
---

```
Periode = 1.9999555s

N_Perioden = 1 / (2s - 1.9999555s)
           = 22471.91011236
           = 22472
```

<!-- end_slide -->

Sleep Mode
---

<!-- column_layout: [1, 1] -->

<!-- column: 0 -->

# Display

1.8mA - 22mA

# Sleep (Power Save)

0.95µA

# Laufzeit

230mAh / 0.95µA = 242105.2h = 27.6years

<!-- column: 1 -->

- https://godbolt.org/z/EbE185oYa

----

- ca. 30 instructions im hot path
- Atmega48a hat 1MIPS pro MHz
- ISR Timer2 braucht ca. (30 / 1000000)s = 0.0003s
- hat kaum Einfluss auf den Stromverbrauch
