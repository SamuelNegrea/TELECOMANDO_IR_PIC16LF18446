/*
 * TELECOMANDO_IR - PIC16LF18446
 * MPLAB X + XC8
 *
 * PROTOTIPO NON VALIDATO SU HARDWARE.
 *
 * Portante IR:
 * - PWM hardware PWM6
 * - Timer2
 * - Frequenza teorica: 36.364 kHz
 * - HIGH: 5,50 us
 * - Duty cycle: 20,00%
 *
 * GPIO temporanei:
 * - Righe tastiera: RC0..RC7
 * - Colonne tastiera: RB4..RB7
 * - LED IR: RA2 e RA4
 *
 * I due LED ricevono lo stesso segnale PWM6.
 */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#define _XTAL_FREQ              32000000UL

/* =========================================================
   PARAMETRI PROTOCOLLO
   ========================================================= */

#define SENDER_ID               1u
#define STATUS_DEFAULT          0u

#define IR_START_CYCLES         68u
#define IR_BIT0_CYCLES          22u
#define IR_BIT1_CYCLES          44u

#define IR_BLANK_US             600u

#define IR_REPEAT_PERIOD_MS     45u
#define KEY_DEBOUNCE_MS         20u

#define NO_KEY                  0xFFu
#define KEY_COUNT               32u

/* =========================================================
   PARAMETRI PWM HARDWARE
   ========================================================= */

/*
 * Fosc = 32 MHz
 *
 * Periodo PWM:
 * T = (T2PR + 1) * 4 / Fosc
 *
 * T2PR = 219:
 * T = 220 * 4 / 32.000.000
 * T = 27,50 us
 *
 * Frequenza = 36.364 kHz
 */

#define IR_PWM_PR2              219u

/*
 * PWMDC = 176
 *
 * Impulso HIGH:
 * 176 / 32 MHz = 5,50 us
 *
 * Duty:
 * 176 / (4 * 220) = 20,00%
 */

#define IR_PWM_DUTY_COUNTS      176u

/*
 * Timer2 incrementa ogni 0,125 us.
 * PWMDC 176 corrisponde a 44 conteggi Timer2.
 */

#define IR_PWM_END_TMR2_COUNT   44u

/* =========================================================
   VARIABILI DEBUG
   ========================================================= */

volatile uint16_t debug_last_frame = 0u;
volatile uint16_t debug_tx_count = 0u;
volatile uint8_t debug_last_key = NO_KEY;
volatile uint8_t debug_last_crc = 0u;

/* =========================================================
   CLOCK
   ========================================================= */

static void clock_init(void)
{
    /*
     * NOSC = 110: HFINTOSC
     * NDIV = 0000: divisore 1:1
     */
    OSCCON1 = 0x60u;

    /*
     * HFFRQ = 0111: HFINTOSC 32 MHz
     */
    OSCFRQ = 0x07u;

    __delay_ms(1);
}

/* =========================================================
   GPIO
   ========================================================= */

static void gpio_init(void)
{
    /* Tutti i pin utilizzati come digitali */
    ANSELA = 0x00u;
    ANSELB = 0x00u;
    ANSELC = 0x00u;

    /*
     * RA2 e RA4 saranno pilotati dal PWM.
     * Li lasciamo inizialmente come ingressi.
     */
    TRISAbits.TRISA2 = 1;
    TRISAbits.TRISA4 = 1;

    LATAbits.LATA2 = 0;
    LATAbits.LATA4 = 0;

    /* Righe tastiera RC0..RC7 */
    TRISC = 0x00u;
    LATC = 0xFFu;

    /* Colonne tastiera RB4..RB7 */
    TRISB = 0xF0u;

    /* Pull-up interne sulle colonne */
    WPUB = 0xF0u;

    /* IOC inizialmente disabilitato */
    IOCBP = 0x00u;
    IOCBN = 0x00u;
    IOCBF = 0x00u;

    PIE0bits.IOCIE = 0;
}

/* =========================================================
   INIZIALIZZAZIONE PWM6 E TIMER2
   ========================================================= */

static void ir_pwm_init(void)
{
    PWM6CONbits.EN = 0;
    PWM6CONbits.POL = 0;
    T2CON = 0x00u;
    CCPTMRS1bits.P6TSEL = 0b01; /* PWM6 usa Timer2 */
    T2CLKCON = 0x01u;           /* Clock = Fosc/4 */
    T2HLT = 0x00u;              /* Free-running, software gate */
    T2RST = 0x00u;
    T2PR = IR_PWM_PR2;
    PWM6DCH = (uint8_t)(IR_PWM_DUTY_COUNTS >> 2);
    PWM6DCL = (uint8_t)((IR_PWM_DUTY_COUNTS & 0x03u) << 6);
    T2TMR = 0u;

    /* PWM6OUT = 0x0D su RA2 e RA4 tramite PPS */
    INTCONbits.GIE = 0;
    PPSLOCK = 0x55u;
    PPSLOCK = 0xAAu;
    PPSLOCKbits.PPSLOCKED = 0;
    RA2PPS = 0x0Du;
    RA4PPS = 0x0Du;
    PPSLOCK = 0x55u;
    PPSLOCK = 0xAAu;
    PPSLOCKbits.PPSLOCKED = 1;

    TRISAbits.TRISA2 = 0;
    TRISAbits.TRISA4 = 0;
    LATAbits.LATA2 = 0;
    LATAbits.LATA4 = 0;
    PWM6CONbits.EN = 0;
    T2CONbits.ON = 0;
}

/* =========================================================
   CRC-4
   Polinomio: x^4 + x + 1
   Init = 0
   Elaborazione LSB-first
   ========================================================= */

static uint8_t crc4_calculate(uint16_t payload12)
{
    uint8_t crc = 0u;
    uint8_t bit_index;
    uint8_t feedback;

    for (bit_index = 0u; bit_index < 12u; bit_index++)
    {
        feedback = (uint8_t)
                   ((crc ^ (payload12 >> bit_index)) & 0x01u);

        crc >>= 1;

        if (feedback != 0u)
        {
            crc ^= 0x0Cu;
        }
    }

    return (uint8_t)(crc & 0x0Fu);
}

/* =========================================================
   COSTRUZIONE PAYLOAD
   ========================================================= */

static uint16_t build_payload(uint8_t sender,
                              uint8_t command,
                              uint8_t status)
{
    uint16_t payload = 0u;

    /* Sender: 3 bit */
    payload |= (uint16_t)(sender & 0x07u);

    /* Command: 6 bit */
    payload |= (uint16_t)
               ((uint16_t)(command & 0x3Fu) << 3);

    /* Status: 3 bit */
    payload |= (uint16_t)
               ((uint16_t)(status & 0x07u) << 9);

    return payload;
}

/* =========================================================
   COSTRUZIONE FRAME
   ========================================================= */

static uint16_t build_frame(uint8_t sender,
                            uint8_t command,
                            uint8_t status)
{
    uint16_t payload;
    uint8_t crc;

    payload = build_payload(sender, command, status);

    crc = crc4_calculate(payload);

    debug_last_crc = crc;

    return (uint16_t)
           (payload | ((uint16_t)crc << 12));
}

/* =========================================================
   GENERAZIONE BURST PWM
   ========================================================= */

static void ir_carrier_burst(uint8_t cycles)
{
    uint8_t i;
    if (cycles == 0u) return;

    PWM6CONbits.EN = 0;
    T2CONbits.ON = 0;
    T2TMR = 0u;
    PWM6CONbits.EN = 1;
    T2CON = 0x80u; /* Timer2 ON, prescaler/postscaler 1:1 */

    for (i = 0u; i < cycles; i++)
    {
        while (T2TMR < IR_PWM_END_TMR2_COUNT) { ; }
        if ((uint8_t)(i + 1u) < cycles)
        {
            while (T2TMR >= IR_PWM_END_TMR2_COUNT) { ; }
        }
    }

    PWM6CONbits.EN = 0;
    T2CONbits.ON = 0;
}

/* =========================================================
   INVIO SINGOLO BIT
   ========================================================= */

static void ir_send_bit(uint8_t bit_value)
{
    if (bit_value != 0u)
    {
        ir_carrier_burst(IR_BIT1_CYCLES);
    }
    else
    {
        ir_carrier_burst(IR_BIT0_CYCLES);
    }

    /* Pausa tra i bit */
    __delay_us(IR_BLANK_US);
}

/* =========================================================
   INVIO FRAME COMPLETO
   ========================================================= */

static void ir_transmit_frame(uint16_t frame)
{
    uint8_t bit_index;

    /* Start burst */
    ir_carrier_burst(IR_START_CYCLES);

    __delay_us(IR_BLANK_US);

    /* Trasmissione dei 16 bit LSB-first */
    for (bit_index = 0u; bit_index < 16u; bit_index++)
    {
        ir_send_bit((uint8_t)
                    ((frame >> bit_index) & 0x01u));
    }

    /* Aggiornamento debug */
    debug_last_frame = frame;
    debug_tx_count++;
}

/* =========================================================
   SCANSIONE TASTIERA 8 x 4
   ========================================================= */

static uint8_t keypad_scan_raw(void)
{
    uint8_t row;
    uint8_t columns;
    uint8_t col;

    /* Disabilita IOC durante la scansione */
    PIE0bits.IOCIE = 0;

    IOCBN = 0x00u;
    IOCBP = 0x00u;

    /* Cancella i flag IOC */
    IOCBF = 0x00u;

    for (row = 0u; row < 8u; row++)
    {
        LATC = 0xFFu;

        /* Porta bassa la riga selezionata */
        LATC &= (uint8_t)~(1u << row);

        __delay_us(50);

        /* Legge RB4..RB7 */
        columns = (uint8_t)((PORTB >> 4) & 0x0Fu);

        if (columns != 0x0Fu)
        {
            for (col = 0u; col < 4u; col++)
            {
                if ((columns & (uint8_t)(1u << col)) == 0u)
                {
                    LATC = 0xFFu;

                    return (uint8_t)(row * 4u + col);
                }
            }
        }
    }

    LATC = 0xFFu;

    return NO_KEY;
}

/* =========================================================
   DEBOUNCE PRESSIONE
   ========================================================= */

static uint8_t keypad_read_debounced(void)
{
    uint8_t first;
    uint8_t second;

    first = keypad_scan_raw();

    __delay_ms(KEY_DEBOUNCE_MS);

    second = keypad_scan_raw();

    if (first == second)
    {
        return first;
    }

    return NO_KEY;
}

/* =========================================================
   ATTESA RILASCIO STABILE
   ========================================================= */

static void keypad_wait_release(void)
{
    uint8_t first;
    uint8_t second;

    do
    {
        first = keypad_scan_raw();

        if (first == NO_KEY)
        {
            __delay_ms(KEY_DEBOUNCE_MS);

            second = keypad_scan_raw();

            if (second == NO_KEY)
            {
                break;
            }
        }

        __delay_ms(10);

    } while (1);
}

/* =========================================================
   SLEEP E RISVEGLIO TRAMITE IOC
   ========================================================= */

static void keypad_sleep_until_press(void)
{
    /* Tutte le righe basse */
    LATC = 0x00u;

    /* Se un tasto Ã¨ giÃ  premuto, non entra in Sleep */
    if ((PORTB & 0xF0u) != 0xF0u)
    {
        LATC = 0xFFu;
        return;
    }

    /* Disabilita i fronti positivi */
    IOCBP = 0x00u;

    /* Abilita i fronti negativi RB4..RB7 */
    IOCBN = 0xF0u;

    /* Cancella i vecchi flag */
    IOCBF = 0x00u;

    /* Abilita IOC */
    PIE0bits.IOCIE = 1;

    /* Abilita interrupt periferici */
    INTCONbits.PEIE = 1;

    /*
     * GIE disabilitato:
     * il PIC si risveglia senza entrare in una ISR.
     */
    INTCONbits.GIE = 0;

    /* Secondo controllo prima dello Sleep */
    if ((PORTB & 0xF0u) != 0xF0u)
    {
        PIE0bits.IOCIE = 0;

        IOCBN = 0x00u;
        IOCBF = 0x00u;

        LATC = 0xFFu;

        return;
    }

    /* ModalitÃ  Sleep */
    SLEEP();
    NOP();

    /* Disabilita IOC dopo il risveglio */
    PIE0bits.IOCIE = 0;

    IOCBN = 0x00u;
    IOCBF = 0x00u;

    /* Ripristina le righe */
    LATC = 0xFFu;
}

/* =========================================================
   TRASMISSIONE DURANTE LA PRESSIONE
   ========================================================= */

static void process_key(uint8_t key)
{
    uint16_t frame;
    uint8_t slice;

    frame = build_frame(SENDER_ID, key, STATUS_DEFAULT);

    debug_last_key = key;

    /* Primo telegramma immediato */
    ir_transmit_frame(frame);

    /*
     * Ripete il telegramma senza un limite massimo di tempo.
     * La trasmissione continua finchÃ© il tasto resta premuto.
     * Il rilascio viene verificato ogni 5 ms e confermato
     * dopo KEY_DEBOUNCE_MS per evitare falsi rilasci.
     */
    while (1)
    {
        for (slice = 0u;
             slice < IR_REPEAT_PERIOD_MS;
             slice += 5u)
        {
            __delay_ms(5);

            /* Controlla che il pulsante sia ancora premuto */
            if (keypad_scan_raw() != key)
            {
                __delay_ms(KEY_DEBOUNCE_MS);

                if (keypad_scan_raw() != key)
                {
                    return;
                }
            }
        }

        /* Trascorsi 45 ms, trasmette un nuovo telegramma */
        ir_transmit_frame(frame);
    }
}

/* =========================================================
   MAIN
   ========================================================= */

void main(void)
{
    uint8_t key;

    clock_init();

    gpio_init();

    /* Inizializza PWM6 e instradamento PPS */
    ir_pwm_init();

    while (1)
    {
        /* Attesa pressione */
        keypad_sleep_until_press();

        /* Lettura con debounce */
        key = keypad_read_debounced();

        if (key < KEY_COUNT)
        {
            process_key(key);
        }

        /* Attende rilascio stabile */
        keypad_wait_release();

        __delay_ms(30);
    }
}
