/*
 * TELECOMANDO_IR - PIC16F18446 (Curiosity Nano)
 * MPLAB X + XC8
 *
 * CLOCK:
 * - HFINTOSC 32 MHz
 *
 * USCITA IR:
 * - PWM6 hardware
 * - Timer2
 * - PWM6OUT assegnato a RA1 (DBG1 / ICSPCLK)
 *
 * PORTANTE:
 * - Periodo: 28.75 us
 * - Frequenza: 34.783 kHz
 * - HIGH: 5.75 us
 * - Duty cycle: 20%
 *
 * PROTOCOLLO:
 * - START: 69 cicli
 * - BIT 0: 22 cicli
 * - BIT 1: 44 cicli
 * - BLANK: 600 us
 * - Ripetizione: una trama ogni 500 ms (inizio-inizio)
 *
 * TASTIERA 8 righe x 4 colonne:
 * - Righe (uscite): RC0, RC1, RC2, RC3, RC4, RC5, RC6, RC7
 * - Colonne (ingressi con pull-up): RB4, RB5, RB6, RB7
 * - Tasto = riga * 4 + colonna (colonna 0 = RB4 ... 3 = RB7)
 *
 * PULSANTE DI TEST:
 * - RA0 (DBG0), attivo LOW, pull-up interna
 *   (abilitato con ENABLE_BUTTON_SCOPE_TEST)
 *
 * USCITA LED IR:
 * - RA1 (DBG1) = PWM6
 *
 * PIN LIBERI:
 * - RA4 (AN4) e RA5 (AN5): analogici, configurati come
 *   ingressi analogici
 * - RA2: libero (collegato a LED0 della scheda), ingresso
 * - RA3: solo ingresso (MCLR)
 *
 * ATTENZIONE (Curiosity Nano):
 * - RA1 e RA0 sono condivisi con ICSPCLK / ICSPDAT.
 *   Programmare il PIC e testare senza debugger attivo.
 * - RC2 (riga 2) e' collegato a SW0 della scheda: non
 *   premerlo durante l'uso della tastiera.
 * - RB4 e RB6 (colonne 0 e 2) sono collegati alla porta
 *   seriale del debugger: eventuale traffico seriale
 *   puo' simulare la pressione di un tasto.
 *
 * PROTOTIPO: verificare temporizzazioni e uscita
 * PWM con oscilloscopio prima del collegamento del LED.
 */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

/* =========================================================
   CONFIGURAZIONE GENERALE
   ========================================================= */

#define _XTAL_FREQ                 32000000UL

/* =========================================================
   PARAMETRI PROTOCOLLO IR
   ========================================================= */

#define SENDER_ID                  1u
#define STATUS_DEFAULT             0u

#define IR_START_CYCLES            69u
#define IR_BIT0_CYCLES             22u
#define IR_BIT1_CYCLES             44u

#define IR_BLANK_US                600u

#define KEY_DEBOUNCE_MS            20u

#define NO_KEY                     0xFFu
#define KEY_COUNT                  32u

#define MIN_FRAMES_PER_PRESS       5u

/* 1 = pulsante di test su RA0 attivo, 0 = escluso */
#define ENABLE_BUTTON_SCOPE_TEST   1u

/* Trama inviata dal pulsante di test RA0 */
#define TEST_FRAME                 0xCAA9u

/* =========================================================
   TABELLA COMANDI: un messaggio diverso per ogni tasto
   ========================================================= */

/*
 * Indice tasto = riga * 4 + colonna  (0..31)
 *
 * Il campo COMMAND del frame e' di 6 bit (0..63).
 * I 32 codici usati sono tutti i valori a 6 bit con
 * numero PARI di bit a 1: due comandi qualsiasi differiscono
 * quindi in almeno 2 bit, e un errore su un solo bit non
 * puo' trasformare un comando in un altro valido
 * (protezione in piu' rispetto al CRC-4).
 *
 * Il codice 0x00 e' assegnato per ultimo (tasto 31).
 */

static const uint8_t key_command[KEY_COUNT] =
{
    /* Riga 0 */
    0x03u,  /* tasto  0 */
    0x05u,  /* tasto  1 */
    0x06u,  /* tasto  2 */
    0x09u,  /* tasto  3 */

    /* Riga 1 */
    0x0Au,  /* tasto  4 */
    0x0Cu,  /* tasto  5 */
    0x0Fu,  /* tasto  6 */
    0x11u,  /* tasto  7 */

    /* Riga 2 */
    0x12u,  /* tasto  8 */
    0x14u,  /* tasto  9 */
    0x17u,  /* tasto 10 */
    0x18u,  /* tasto 11 */

    /* Riga 3 */
    0x1Bu,  /* tasto 12 */
    0x1Du,  /* tasto 13 */
    0x1Eu,  /* tasto 14 */
    0x21u,  /* tasto 15 */

    /* Riga 4 */
    0x22u,  /* tasto 16 */
    0x24u,  /* tasto 17 */
    0x27u,  /* tasto 18 */
    0x28u,  /* tasto 19 */

    /* Riga 5 */
    0x2Bu,  /* tasto 20 */
    0x2Du,  /* tasto 21 */
    0x2Eu,  /* tasto 22 */
    0x30u,  /* tasto 23 */

    /* Riga 6 */
    0x33u,  /* tasto 24 */
    0x35u,  /* tasto 25 */
    0x36u,  /* tasto 26 */
    0x39u,  /* tasto 27 */

    /* Riga 7 */
    0x3Au,  /* tasto 28 */
    0x3Cu,  /* tasto 29 */
    0x3Fu,  /* tasto 30 */
    0x00u   /* tasto 31 */
};

/* =========================================================
   MASCHERE PIN TASTIERA
   ========================================================= */

/* Righe: RC0..RC7 (PORTC) */
#define ROWS_MASK_C                0xFFu

/* Colonne: RB4..RB7 (PORTB) */
#define COLS_MASK_B                0xF0u

/* =========================================================
   PARAMETRI PWM6 / TIMER2
   ========================================================= */

/*
 * Fosc = 32 MHz
 * Fosc/4 = 8 MHz
 * Tick Timer2 = 125 ns
 *
 * T2PR = 229
 *
 * Periodo:
 * (229 + 1) / 8 MHz = 28.75 us
 *
 * Frequenza:
 * 1 / 28.75 us = 34.7826 kHz
 *
 * Risoluzione PWM:
 * 4 * (T2PR + 1) = 920 conteggi
 *
 * Duty:
 * 184 / 920 = 20%
 *
 * HIGH:
 * 184 * 31.25 ns = 5.75 us
 */

#define IR_PWM_PR2                 229u
#define IR_PWM_DUTY_COUNTS         184u

/*
 * Timer2:
 * 1 conteggio = 125 ns
 *
 * 46 conteggi = 5.75 us
 */

#define IR_PWM_END_TMR2_COUNT      46u

/* =========================================================
   TIMER0 - PERIODO DI RIPETIZIONE (500 ms)
   ========================================================= */

/*
 * Fosc/4 = 8 MHz
 * Prescaler 1:64 = 125 kHz (8 us per conteggio)
 *
 * 500 ms = 62500 conteggi
 *
 * 65536 - 62500 = 3036 = 0x0BDC
 */

#define TMR0_PRELOAD               0x0BDCu

/* =========================================================
   VARIABILI DEBUG
   ========================================================= */

volatile uint16_t debug_last_frame = 0u;
volatile uint16_t debug_tx_count = 0u;

volatile uint8_t debug_last_key = NO_KEY;
volatile uint8_t debug_last_command = 0u;
volatile uint8_t debug_last_crc = 0u;

/* =========================================================
   INIZIALIZZAZIONE CLOCK
   ========================================================= */

static void clock_init(void)
{
    /*
     * NOSC = HFINTOSC
     * NDIV = 1:1
     */

    OSCCON1 = 0x60u;

    /*
     * HFINTOSC = 32 MHz
     */

    OSCFRQ = 0x07u;

    __delay_ms(1);
}

/* =========================================================
   INIZIALIZZAZIONE GPIO
   ========================================================= */

static void gpio_init(void)
{
    /*
     * Modalita' analogica / digitale
     *
     * PORTA: RA4 e RA5 analogici (AN4, AN5),
     *        gli altri digitali
     * PORTB, PORTC: tutti digitali
     */

    ANSELA = 0x30u;
    ANSELB = 0x00u;
    ANSELC = 0x00u;

    /*
     * Livelli iniziali:
     * - righe tutte HIGH (inattive)
     * - RA1 (PWM) LOW
     */

    LATA = 0x00u;
    LATB = 0x00u;
    LATC = ROWS_MASK_C;

    /*
     * PORTA:
     * RA0 = ingresso (pulsante di test, pull-up)
     * RA1 = uscita (PWM6 -> LED IR)
     * RA2 = ingresso (libero)
     * RA3 = ingresso (MCLR)
     * RA4 = ingresso analogico
     * RA5 = ingresso analogico
     */

    TRISA = 0x3Du;

    /*
     * PORTB:
     * RB4..RB7 = ingressi (colonne)
     */

    TRISB = COLS_MASK_B;

    /*
     * PORTC:
     * RC0..RC7 = uscite (righe)
     */

    TRISC = 0x00u;

    /*
     * Pull-up sulle colonne RB4..RB7
     * e sul pulsante di test RA0
     */

    WPUA = 0x01u;
    WPUB = COLS_MASK_B;
    WPUC = 0x00u;

    /*
     * IOC disabilitato
     */

    IOCAP = 0x00u;
    IOCAN = 0x00u;
    IOCAF = 0x00u;

    IOCBP = 0x00u;
    IOCBN = 0x00u;
    IOCBF = 0x00u;

    IOCCP = 0x00u;
    IOCCN = 0x00u;
    IOCCF = 0x00u;

    PIE0bits.IOCIE = 0;
}

/* =========================================================
   INIZIALIZZAZIONE PWM6 E TIMER2
   ========================================================= */

static void ir_pwm_init(void)
{
    /*
     * Disabilita PWM6
     */

    PWM6CONbits.EN = 0;

    /*
     * Polarita' normale
     */

    PWM6CONbits.POL = 0;

    /*
     * Timer2 spento durante la configurazione
     */

    T2CON = 0x00u;

    /*
     * PWM6 utilizza Timer2.
     * P6TSEL = 01
     */

    CCPTMRS1bits.P6TSEL = 0b01;

    /*
     * Clock Timer2 = Fosc/4
     */

    T2CLKCON = 0x01u;

    /*
     * Modalita' Free Running
     */

    T2HLT = 0x00u;

    /*
     * Nessuna sorgente esterna di reset
     */

    T2RST = 0x00u;

    /*
     * Periodo PWM
     */

    T2PR = IR_PWM_PR2;

    /*
     * Duty PWM a 10 bit
     */

    PWM6DCH =
        (uint8_t)(IR_PWM_DUTY_COUNTS >> 2);

    PWM6DCL =
        (uint8_t)((IR_PWM_DUTY_COUNTS & 0x03u) << 6);

    /*
     * Contatore iniziale
     */

    T2TMR = 0u;

    /*
     * Configurazione PPS:
     * PWM6OUT -> RA1
     *
     * 0x0D = PWM6OUT
     */

    INTCONbits.GIE = 0;

    PPSLOCK = 0x55u;
    PPSLOCK = 0xAAu;

    PPSLOCKbits.PPSLOCKED = 0;

    RA1PPS = 0x0Du;

    PPSLOCK = 0x55u;
    PPSLOCK = 0xAAu;

    PPSLOCKbits.PPSLOCKED = 1;

    /*
     * RA1 come uscita
     */

    TRISAbits.TRISA1 = 0;

    /*
     * Latch iniziale LOW
     */

    LATAbits.LATA1 = 0;

    /*
     * PWM6 e Timer2 rimangono spenti.
     * Verranno attivati durante i burst.
     */

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

    /*
     * Sender: 3 bit
     * Bit 0..2
     */

    payload |= (uint16_t)(sender & 0x07u);

    /*
     * Command: 6 bit
     * Bit 3..8
     */

    payload |= (uint16_t)
        ((uint16_t)(command & 0x3Fu) << 3);

    /*
     * Status: 3 bit
     * Bit 9..11
     */

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

    /*
     * CRC nei bit 12..15
     */

    return (uint16_t)
        (payload | ((uint16_t)crc << 12));
}

/* =========================================================
   GENERAZIONE BURST PWM
   ========================================================= */

/*
 * Genera un burst di N periodi PWM.
 *
 * Il flag TMR2IF segnala il match tra T2TMR e T2PR.
 *
 * La funzione:
 * 1. Azzera Timer2.
 * 2. Avvia PWM6 e Timer2.
 * 3. Conta N-1 periodi completi.
 * 4. Attende la fine della parte HIGH dell'ultimo periodo.
 * 5. Disabilita PWM6 e Timer2.
 *
 * La durata reale e il bordo di spegnimento devono essere
 * verificati con l'oscilloscopio.
 */

static void ir_carrier_burst(uint8_t cycles)
{
    uint8_t i;

    if (cycles == 0u)
    {
        return;
    }

    /*
     * Ferma PWM6 e Timer2
     */

    PWM6CONbits.EN = 0;
    T2CONbits.ON = 0;

    /*
     * Azzera Timer2
     */

    T2TMR = 0u;

    /*
     * Cancella il flag Timer2
     */

    PIR4bits.TMR2IF = 0;

    /*
     * Abilita PWM6
     */

    PWM6CONbits.EN = 1;

    /*
     * Avvia Timer2
     */

    T2CONbits.ON = 1;

    /*
     * Attende N-1 periodi completi.
     */

    for (i = 1u; i < cycles; i++)
    {
        /*
         * Attende il match TMR2 = PR2
         */

        while (PIR4bits.TMR2IF == 0)
        {
            ;
        }

        /*
         * Cancella il flag
         */

        PIR4bits.TMR2IF = 0;

        /*
         * Attende il riavvio del contatore.
         */

        while (T2TMR >= IR_PWM_PR2)
        {
            ;
        }
    }

    /*
     * Attende la fine della parte HIGH
     * dell'ultimo periodo.
     *
     * 46 conteggi * 125 ns = 5.75 us
     */

    while (T2TMR < IR_PWM_END_TMR2_COUNT)
    {
        ;
    }

    /*
     * Ferma il PWM.
     * Il latch di RA1 e' mantenuto LOW.
     */

    PWM6CONbits.EN = 0;

    LATAbits.LATA1 = 0;

    /*
     * Ferma Timer2
     */

    T2CONbits.ON = 0;

    /*
     * Cancella il flag
     */

    PIR4bits.TMR2IF = 0;
}

/* =========================================================
   INVIO SINGOLO BIT
   ========================================================= */

static void ir_send_bit(uint8_t bit_value)
{
    if (bit_value == 0u)
    {
        /*
         * BIT 0:
         * 22 periodi PWM
         */

        ir_carrier_burst(IR_BIT0_CYCLES);
    }
    else
    {
        /*
         * BIT 1:
         * 44 periodi PWM
         */

        ir_carrier_burst(IR_BIT1_CYCLES);
    }

    /*
     * Blank tra i bit:
     * 600 us senza portante
     */

    __delay_us(IR_BLANK_US);
}

/* =========================================================
   INVIO FRAME COMPLETO
   ========================================================= */

static void ir_transmit_frame(uint16_t frame)
{
    uint8_t bit_index;

    /*
     * START:
     * 69 periodi PWM
     */

    ir_carrier_burst(IR_START_CYCLES);

    /*
     * Blank dopo START
     */

    __delay_us(IR_BLANK_US);

    /*
     * Trasmissione dei 16 bit LSB-first
     */

    for (bit_index = 0u; bit_index < 16u; bit_index++)
    {
        ir_send_bit(
            (uint8_t)((frame >> bit_index) & 0x01u)
        );
    }

    debug_last_frame = frame;
    debug_tx_count++;
}

/* =========================================================
   TASTIERA 8 righe x 4 colonne - GESTIONE RIGHE
   ========================================================= */

/*
 * Porta HIGH tutte le righe (inattive).
 */
static void keypad_rows_all_high(void)
{
    LATC = ROWS_MASK_C;
}

/*
 * Porta LOW la riga selezionata (0..7 = RC0..RC7).
 * Va chiamata dopo keypad_rows_all_high().
 */
static void keypad_row_select(uint8_t row)
{
    LATC &= (uint8_t)~(1u << row);
}

/* =========================================================
   SCANSIONE TASTIERA 8 x 4
   ========================================================= */

static uint8_t keypad_scan_raw(void)
{
    uint8_t row;
    uint8_t columns;
    uint8_t col;

    for (row = 0u; row < 8u; row++)
    {
        /*
         * Tutte le righe HIGH, poi LOW quella selezionata
         */

        keypad_rows_all_high();
        keypad_row_select(row);

        __delay_us(50);

        /*
         * Legge RB4..RB7
         */

        columns = (uint8_t)((PORTB >> 4) & 0x0Fu);

        if (columns != 0x0Fu)
        {
            for (col = 0u; col < 4u; col++)
            {
                if ((columns & (uint8_t)(1u << col)) == 0u)
                {
                    keypad_rows_all_high();

                    return (uint8_t)(row * 4u + col);
                }
            }
        }
    }

    keypad_rows_all_high();

    return NO_KEY;
}

/* =========================================================
   DEBOUNCE PRESSIONE TASTIERA
   ========================================================= */

static uint8_t keypad_read_debounced(void)
{
    uint8_t first;
    uint8_t second;
    uint8_t i;

    first = keypad_scan_raw();

    for (i = 0u; i < KEY_DEBOUNCE_MS; i++)
    {
        __delay_ms(1);
    }

    second = keypad_scan_raw();

    if (first == second)
    {
        return first;
    }

    return NO_KEY;
}

/* =========================================================
   ATTESA RILASCIO TASTIERA
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
   TIMER0 - PERIODO DI RIPETIZIONE 500 ms
   ========================================================= */

static void timer0_init(void)
{
    T0CON0bits.T0EN = 0;

    /*
     * Clock: Fosc/4 = 8 MHz
     * Prescaler: 1:64
     * Timer0: 125 kHz
     */
    T0CON1 = 0x46u;

    /* Modalita' 16 bit, postscaler 1:1 */
    T0CON0 = 0x10u;

    TMR0H = (uint8_t)(TMR0_PRELOAD >> 8);
    TMR0L = (uint8_t)TMR0_PRELOAD;

    PIR0bits.TMR0IF = 0;
}

static void timer0_start_period(void)
{
    T0CON0bits.T0EN = 0;

    TMR0H = (uint8_t)(TMR0_PRELOAD >> 8);
    TMR0L = (uint8_t)TMR0_PRELOAD;

    PIR0bits.TMR0IF = 0;

    T0CON0bits.T0EN = 1;
}

/*
 * Attende l'overflow (500 ms dall'avvio) e ferma il timer.
 * Dopo il ritorno sono trascorsi ALMENO 500 ms dall'inizio
 * dell'ultima trama.
 */
static void timer0_wait_period(void)
{
    while (PIR0bits.TMR0IF == 0)
    {
        ;
    }

    T0CON0bits.T0EN = 0;

    PIR0bits.TMR0IF = 0;
}

/* =========================================================
   TRASMISSIONE DA PULSANTE DI TEST RA0
   ========================================================= */

#if ENABLE_BUTTON_SCOPE_TEST

static void process_button_ir(void)
{
    while (1)
    {
        /* Avvia il timer prima della trasmissione */
        timer0_start_period();

        /* Trasmette una sola trama */
        ir_transmit_frame(TEST_FRAME);

        /* Attende 500 ms dall'inizio della trama */
        timer0_wait_period();

        /* Controlla se il pulsante e' stato rilasciato */
        if (PORTAbits.RA0 != 0)
        {
            /* Attende la stabilizzazione del contatto */
            __delay_ms(KEY_DEBOUNCE_MS);

            /* Conferma il rilascio */
            if (PORTAbits.RA0 != 0)
            {
                break;
            }
        }
    }
}

#endif

/* =========================================================
   TRASMISSIONE DA TASTIERA
   ========================================================= */

static void process_key(uint8_t key)
{
    uint16_t frame;
    uint8_t command;
    uint8_t sent;

    /*
     * Messaggio associato al tasto
     */

    command = key_command[key];

    /*
     * Costruisce il frame
     */

    frame = build_frame(
        SENDER_ID,
        command,
        STATUS_DEFAULT
    );

    debug_last_key = key;
    debug_last_command = command;

    sent = 0u;

    for (;;)
    {
        /*
         * Timer0 parte subito prima della trama:
         * il riferimento e' l'inizio della trasmissione.
         */

        timer0_start_period();

        ir_transmit_frame(frame);

        sent++;

        /*
         * Attende SEMPRE la fine dei 500 ms prima di
         * qualsiasi decisione: nessun percorso di uscita
         * permette di ritrasmettere prima del tempo.
         */

        timer0_wait_period();

        /*
         * Dopo almeno MIN_FRAMES_PER_PRESS trame controlla
         * se il tasto e' stato rilasciato, a fine periodo.
         */

        if (sent >= MIN_FRAMES_PER_PRESS)
        {
            if (keypad_scan_raw() != key)
            {
                __delay_ms(KEY_DEBOUNCE_MS);

                if (keypad_scan_raw() != key)
                {
                    break;
                }
            }
        }
    }
}

/* =========================================================
   MAIN
   ========================================================= */

void main(void)
{
    uint8_t key;

    /*
     * Inizializzazione
     */

    clock_init();

    gpio_init();

    ir_pwm_init();

    timer0_init();

    /*
     * Ciclo principale
     */

    while (1)
    {
        /*
         * PULSANTE DI TEST RA0
         */

#if ENABLE_BUTTON_SCOPE_TEST

        if (PORTAbits.RA0 == 0)
        {
            __delay_ms(KEY_DEBOUNCE_MS);

            if (PORTAbits.RA0 == 0)
            {
                process_button_ir();

                continue;
            }
        }

#endif

        /*
         * TASTIERA 8 x 4
         */

        key = keypad_read_debounced();

        if (key < KEY_COUNT)
        {
            process_key(key);
        }

        /*
         * Attende il rilascio della tastiera
         */

        keypad_wait_release();

        __delay_ms(30);
    }
}
