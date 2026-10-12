/*
 * ==========================================================================
 *  PASSWORD-BASED ACCESS CONTROL SYSTEM USING LPC1768
 *  ICT 3144 Embedded Systems Lab - Mini Project
 *  Board : ALS-SDA-ARMCTXM3-01 (LPC1768, ARM Cortex-M3)
 * ==========================================================================
 *
 *  HARDWARE SETUP (three 10-pin FRC cables)
 *    LCD      : CND -> CNAD   short JP16 and JP5, adjust POT3 for contrast
 *    Keyboard : CNB -> CNB3   short JP4 (1,2)
 *    LEDs     : CNA -> CNA1
 *
 *  PORT LINES
 *    LCD data D4-D7  : P0.23 - P0.26     RS : P0.27     EN : P0.28
 *    Keyboard rows   : P2.10 - P2.13 (outputs)
 *    Keyboard columns: P1.23 - P1.26 (inputs)
 *    LEDs            : P0.4  - P0.11
 *
 *  KEYS (key codes 0-F, the same codes the Lab 9 key-code program shows)
 *    0 - 9 : digits
 *    A     : ENTER
 *    B     : BACKSPACE (delete the last digit)
 *    C     : CHANGE PIN (from the home screen)
 *    E     : NEXT entry (while viewing the access log)
 *    F     : CANCEL / EXIT
 *
 *  FEATURES
 *    1. Multiple users  : ID 0 = ADMIN, ID 1-3 = USER1..USER3, each with
 *                         its own 4-digit PIN.
 *    2. Masked PIN entry: digits are shown as '*', with backspace.
 *    3. Granted / denied: LCD message + LED indication.
 *    4. Lockout         : after 3 wrong PINs in a row the keypad is ignored
 *                         and a countdown runs on the LCD (Timer0).
 *    5. Increasing lockout time: 10 s, 20 s, 40 s, 80 s, then 160 s.
 *                         Goes back to 10 s after a correct PIN.
 *    6. Auto-clear      : a half-typed entry is cleared if no key is
 *                         pressed for 10 s.
 *    7. Change PIN      : any user (or admin) can change their own PIN.
 *    8. Access log      : last 10 events (granted, denied, lockout, PIN
 *                         changed, PIN reset) with user and time since
 *                         power-on. Viewed from the admin menu.
 *    9. Admin menu      : view log, reset a user's PIN, clear the log.
 *
 *  DEFAULT PINS : ADMIN 1234, USER1 1111, USER2 2222, USER3 3333
 *  PINs and the log are kept in RAM, so they reset when the board resets.
 * ==========================================================================
 */

#include <LPC17xx.h>

/* ----------------------------- settings ---------------------------------- */
#define NUM_IDS          4       /* ID 0 = admin, ID 1..3 = users          */
#define PIN_LEN          4       /* digits in a PIN                        */
#define MAX_TRIES        3       /* wrong PINs in a row before lockout     */
#define LOCK_BASE_SEC    10      /* length of the first lockout (seconds)  */
#define LOCK_MAX_LEVEL   4       /* 10,20,40,80,160 s, then stays at 160 s */
#define IDLE_TIMEOUT_MS  10000   /* auto-clear after 10 s with no key      */
#define LOG_SIZE         10      /* number of events kept in the log       */

/* Set to 1 to only show the code of each key pressed (to check the keypad
   before running the project). Set back to 0 for the real program.        */
#define KEY_TEST_MODE    0

/* ----------------------------- key codes --------------------------------- */
#define KEY_ENTER   0x0A
#define KEY_BACK    0x0B
#define KEY_CHANGE  0x0C
#define KEY_NEXT    0x0E
#define KEY_CANCEL  0x0F
#define KEY_NONE    0xFF         /* no key pressed / timed out             */

/* ----------------------------- pin masks --------------------------------- */
#define LCD_DATA    (0x0FUL << 23)   /* P0.23 - P0.26 */
#define LCD_RS      (1UL << 27)      /* P0.27         */
#define LCD_EN      (1UL << 28)      /* P0.28         */
#define LED_MASK    (0xFFUL << 4)    /* P0.4  - P0.11 */
#define ROW_MASK    (0x0FUL << 10)   /* P2.10 - P2.13 */
#define COL_MASK    (0x0FUL << 23)   /* P1.23 - P1.26 */

/* Scan code (row * 4 + column) -> key value. If a key shows the wrong code
   on your board (check with KEY_TEST_MODE), correct this table.           */
static const unsigned char keymap[16] = {
    0x0, 0x1, 0x2, 0x3,
    0x4, 0x5, 0x6, 0x7,
    0x8, 0x9, 0xA, 0xB,
    0xC, 0xD, 0xE, 0xF
};

/* ----------------------------- users ------------------------------------- */
static const char default_pins[NUM_IDS][PIN_LEN + 1] = { "1234", "1111", "2222", "3333" };
static char pins[NUM_IDS][PIN_LEN + 1]               = { "1234", "1111", "2222", "3333" };

static unsigned char wrong_tries = 0;   /* wrong PINs in a row            */
static unsigned char lock_level  = 0;   /* how many lockouts in a row     */

/* ----------------------------- access log -------------------------------- */
#define EV_GRANTED  0
#define EV_DENIED   1
#define EV_LOCKOUT  2
#define EV_PIN_CHG  3
#define EV_PIN_RST  4

static const char *const ev_name[] = { "GRANTED", "DENIED ", "LOCKOUT", "PIN CHG", "PIN RST" };

typedef struct {
    unsigned char event;
    unsigned char id;
    uint32_t      sec;        /* seconds since power-on */
} log_entry_t;

static log_entry_t   log_buf[LOG_SIZE];
static unsigned char log_head  = 0;     /* next slot to write             */
static unsigned char log_count = 0;     /* number of valid entries        */

/* result of an input step */
#define IN_OK       0
#define IN_CANCEL   1
#define IN_TIMEOUT  2

/* ----------------------------- timer ------------------------------------- */
volatile uint32_t ms_ticks = 0;         /* incremented every 1 ms         */

void TIMER0_IRQHandler(void)
{
    LPC_TIM0->IR = 1;                   /* clear the MR0 interrupt flag   */
    ms_ticks++;
}

static void timer0_init(void)
{
    LPC_SC->PCONP |= (1UL << 1);        /* power on Timer0                */
    LPC_TIM0->TCR = 0x02;               /* reset counter                  */
    LPC_TIM0->PR  = 0;                  /* no prescale                    */
    /* Timer0 clock = CCLK / 4 (default), so this gives a 1 ms interrupt  */
    LPC_TIM0->MR0 = (SystemCoreClock / 4) / 1000 - 1;
    LPC_TIM0->MCR = 0x03;               /* interrupt and reset on MR0     */
    LPC_TIM0->TCR = 0x01;               /* start                          */
    NVIC_EnableIRQ(TIMER0_IRQn);
}

static void delay_ms(uint32_t ms)
{
    uint32_t start = ms_ticks;
    while ((ms_ticks - start) < ms)
        ;
}

static void delay_loop(volatile uint32_t n)
{
    while (n--)
        ;
}

/* ----------------------------- GPIO -------------------------------------- */
static void gpio_init(void)
{
    LPC_PINCON->PINSEL0 &= ~(0xFFFFUL << 8);   /* P0.4  - P0.11 as GPIO   */
    LPC_PINCON->PINSEL1 &= ~(0x0FFFUL << 14);  /* P0.23 - P0.28 as GPIO   */
    LPC_PINCON->PINSEL3 &= ~(0x00FFUL << 14);  /* P1.23 - P1.26 as GPIO   */
    LPC_PINCON->PINSEL4 &= ~(0x00FFUL << 20);  /* P2.10 - P2.13 as GPIO   */
    LPC_PINCON->PINMODE3 |= (0x00FFUL << 14);  /* columns: pull-down      */

    LPC_GPIO0->FIODIR |= LED_MASK | LCD_DATA | LCD_RS | LCD_EN;  /* outputs */
    LPC_GPIO2->FIODIR |= ROW_MASK;             /* rows are outputs        */
    LPC_GPIO1->FIODIR &= ~COL_MASK;            /* columns are inputs      */
    LPC_GPIO2->FIOCLR  = ROW_MASK;
}

/* ----------------------------- LEDs -------------------------------------- */
static void leds(unsigned char pattern)
{
    LPC_GPIO0->FIOCLR = LED_MASK;
    LPC_GPIO0->FIOSET = ((uint32_t)pattern) << 4;
}

static void flash_leds(unsigned char times)
{
    while (times--) {
        leds(0xFF); delay_ms(200);
        leds(0x00); delay_ms(200);
    }
}

/* ----------------------------- LCD (4-bit mode) -------------------------- */
/* Only FIOSET/FIOCLR are used here (never FIOPIN), so the LEDs, which are
   on the same port, are not disturbed by LCD writes.                       */
static void lcd_nibble(unsigned char nib, unsigned char rs)
{
    LPC_GPIO0->FIOCLR = LCD_DATA;
    LPC_GPIO0->FIOSET = ((uint32_t)(nib & 0x0F)) << 23;
    if (rs)
        LPC_GPIO0->FIOSET = LCD_RS;
    else
        LPC_GPIO0->FIOCLR = LCD_RS;
    LPC_GPIO0->FIOSET = LCD_EN;
    delay_loop(50);
    LPC_GPIO0->FIOCLR = LCD_EN;
    delay_loop(3000);
}

static void lcd_byte(unsigned char b, unsigned char rs)
{
    lcd_nibble(b >> 4, rs);
    lcd_nibble(b & 0x0F, rs);
}

static void lcd_cmd(unsigned char c)
{
    lcd_byte(c, 0);
    if (c == 0x01 || c == 0x02)
        delay_ms(2);                    /* clear / home need extra time   */
}

static void lcd_init(void)
{
    delay_ms(20);
    lcd_nibble(0x3, 0); delay_ms(5);
    lcd_nibble(0x3, 0); delay_ms(1);
    lcd_nibble(0x3, 0); delay_ms(1);
    lcd_nibble(0x2, 0); delay_ms(1);    /* switch to 4-bit mode           */
    lcd_cmd(0x28);                      /* 4-bit, 2 lines, 5x7 font       */
    lcd_cmd(0x0C);                      /* display on, cursor off         */
    lcd_cmd(0x06);                      /* move cursor right              */
    lcd_cmd(0x01);                      /* clear                          */
}

/* Write one full line (row 0 or 1), padded with spaces to 16 characters.
   Overwriting instead of clearing avoids flicker.                         */
static void lcd_line(unsigned char row, const char *s)
{
    unsigned char i;
    lcd_cmd(row == 0 ? 0x80 : 0xC0);
    for (i = 0; i < 16; i++) {
        if (*s)
            lcd_byte((unsigned char)*s++, 1);
        else
            lcd_byte(' ', 1);
    }
}

static void lcd_show(const char *line1, const char *line2)
{
    lcd_line(0, line1);
    lcd_line(1, line2);
}

static void message(const char *line1, const char *line2, uint32_t ms)
{
    lcd_show(line1, line2);
    delay_ms(ms);
}

/* ----------------------------- keypad ------------------------------------ */
/* Drive one row high at a time and read the columns. Returns the key value
   or KEY_NONE.                                                             */
static unsigned char keypad_scan(void)
{
    unsigned char row, col;
    uint32_t cols;

    for (row = 0; row < 4; row++) {
        LPC_GPIO2->FIOCLR = ROW_MASK;
        LPC_GPIO2->FIOSET = 1UL << (10 + row);
        delay_loop(100);                            /* let lines settle   */
        cols = (LPC_GPIO1->FIOPIN & COL_MASK) >> 23;
        if (cols) {
            LPC_GPIO2->FIOCLR = ROW_MASK;
            for (col = 0; col < 4; col++)
                if (cols & (1UL << col))
                    break;
            return keymap[row * 4 + col];
        }
    }
    LPC_GPIO2->FIOCLR = ROW_MASK;
    return KEY_NONE;
}

/* Wait for one debounced key press and its release.
   timeout_ms = 0 waits forever; otherwise returns KEY_NONE if no key is
   pressed within timeout_ms (this is what gives the auto-clear feature).  */
static unsigned char get_key(uint32_t timeout_ms)
{
    uint32_t start = ms_ticks;
    unsigned char k;

    while (1) {
        if (timeout_ms && (ms_ticks - start) >= timeout_ms)
            return KEY_NONE;
        k = keypad_scan();
        if (k != KEY_NONE) {
            delay_ms(20);                           /* debounce press     */
            if (keypad_scan() == k) {
                while (keypad_scan() != KEY_NONE)   /* wait for release   */
                    ;
                delay_ms(20);                       /* debounce release   */
                return k;
            }
        }
    }
}

/* ----------------------------- small helpers ----------------------------- */
/* write v as 'width' digits with leading zeros (no terminator)            */
static void put_num(char *buf, uint32_t v, unsigned char width)
{
    while (width--) {
        buf[width] = (char)('0' + v % 10);
        v /= 10;
    }
}

/* copy src to dst, return pointer to the end of dst (terminated)          */
static char *str_put(char *dst, const char *src)
{
    while (*src)
        *dst++ = *src++;
    *dst = '\0';
    return dst;
}

/* "ADMIN" or "USER1".."USER3" (5 characters)                              */
static void id_name(char *buf, unsigned char id)
{
    if (id == 0) {
        str_put(buf, "ADMIN");
    } else {
        char *p = str_put(buf, "USER");
        p[0] = (char)('0' + id);
        p[1] = '\0';
    }
}

static unsigned char pin_equal(const char *a, const char *b)
{
    unsigned char i;
    for (i = 0; i < PIN_LEN; i++)
        if (a[i] != b[i])
            return 0;
    return 1;
}

/* Shows "TIMED OUT" for a timeout. Returns 1 if the step did not succeed. */
static unsigned char aborted(unsigned char result)
{
    if (result == IN_TIMEOUT)
        message("TIMED OUT", "ENTRY CLEARED", 1500);
    return result != IN_OK;
}

/* ----------------------------- access log -------------------------------- */
static void log_add(unsigned char event, unsigned char id)
{
    log_buf[log_head].event = event;
    log_buf[log_head].id    = id;
    log_buf[log_head].sec   = ms_ticks / 1000;
    log_head = (unsigned char)((log_head + 1) % LOG_SIZE);
    if (log_count < LOG_SIZE)
        log_count++;
}

/* i = 0 is the newest entry                                               */
static log_entry_t *log_get(unsigned char i)
{
    return &log_buf[(log_head + LOG_SIZE - 1 - i) % LOG_SIZE];
}

static unsigned char view_log(void)
{
    unsigned char i = 0, k;
    char l1[17], l2[17];
    log_entry_t *e;

    if (log_count == 0) {
        message("LOG IS EMPTY", "", 1500);
        return IN_OK;
    }
    while (1) {
        e = log_get(i);
        /* line 1: "01 GRANTED USER2" */
        put_num(l1, (uint32_t)i + 1, 2);
        l1[2] = ' ';
        str_put(l1 + 3, ev_name[e->event]);
        l1[10] = ' ';
        id_name(l1 + 11, e->id);
        /* line 2: "T+00245s  E:NEXT" (seconds since power-on) */
        l2[0] = 'T';
        l2[1] = '+';
        put_num(l2 + 2, e->sec, 5);
        l2[7] = 's';
        str_put(l2 + 8, "  E:NEXT");
        lcd_show(l1, l2);

        k = get_key(IDLE_TIMEOUT_MS);
        if (k == KEY_NONE)
            return IN_TIMEOUT;
        if (k == KEY_CANCEL)
            return IN_OK;
        if (k == KEY_NEXT)
            i = (unsigned char)((i + 1) % log_count);
    }
}

/* ----------------------------- input screens ----------------------------- */
/* Read a PIN_LEN-digit PIN. Each digit is shown as '*'.
   B deletes the last digit, A accepts (only when all digits are entered),
   F cancels, 10 s without a key clears the entry.                         */
static unsigned char read_pin(const char *prompt, char *pin)
{
    unsigned char n = 0, i, k;
    char l2[17];

    lcd_line(0, prompt);
    while (1) {
        /* line 2: "PIN: **__  A=OK" */
        str_put(l2, "PIN: ");
        for (i = 0; i < PIN_LEN; i++)
            l2[5 + i] = (i < n) ? '*' : '_';
        str_put(l2 + 5 + PIN_LEN, "  A=OK");
        lcd_line(1, l2);

        k = get_key(IDLE_TIMEOUT_MS);
        if (k == KEY_NONE)
            return IN_TIMEOUT;
        if (k == KEY_CANCEL)
            return IN_CANCEL;
        if (k <= 9 && n < PIN_LEN)
            pin[n++] = (char)('0' + k);
        else if (k == KEY_BACK && n > 0)
            n--;
        else if (k == KEY_ENTER && n == PIN_LEN) {
            pin[n] = '\0';
            return IN_OK;
        }
    }
}

/* Read a single-digit user ID between min_id and NUM_IDS-1.               */
static unsigned char read_id(const char *prompt, unsigned char min_id, unsigned char *id)
{
    unsigned char k;
    char l2[17];

    /* line 2: "ID 0-3   F=BACK" */
    str_put(l2, "ID 0-3   F=BACK");
    l2[3] = (char)('0' + min_id);
    l2[5] = (char)('0' + NUM_IDS - 1);
    lcd_show(prompt, l2);

    while (1) {
        k = get_key(IDLE_TIMEOUT_MS);
        if (k == KEY_NONE)
            return IN_TIMEOUT;
        if (k == KEY_CANCEL)
            return IN_CANCEL;
        if (k >= min_id && k < NUM_IDS) {
            *id = k;
            return IN_OK;
        }
    }
}

/* ----------------------------- results ----------------------------------- */
static void access_granted(unsigned char id)
{
    char l2[17];

    wrong_tries = 0;
    lock_level  = 0;                    /* lockout length back to 10 s    */
    log_add(EV_GRANTED, id);

    str_put(l2, "WELCOME ");
    id_name(l2 + 8, id);
    lcd_show("ACCESS GRANTED", l2);
    leds(0xFF);                         /* all LEDs on = door open        */
    delay_ms(3000);
    leds(0x00);
}

static void lockout(unsigned char id)
{
    uint32_t secs = (uint32_t)LOCK_BASE_SEC << lock_level;
    uint32_t start, elapsed, left, shown = 0xFFFFFFFFUL;
    unsigned char pattern, last_pattern = 0;
    char l2[17];

    if (lock_level < LOCK_MAX_LEVEL)
        lock_level++;                   /* next lockout will be longer    */
    wrong_tries = 0;
    log_add(EV_LOCKOUT, id);

    lcd_line(0, "SYSTEM LOCKED!");
    start = ms_ticks;
    /* The keypad is not read here, so every key is ignored. */
    while ((elapsed = ms_ticks - start) < secs * 1000) {
        left = secs - elapsed / 1000;
        if (left != shown) {            /* update once per second         */
            str_put(l2, "WAIT ");
            put_num(l2 + 5, left, 3);
            str_put(l2 + 8, " SEC");
            lcd_line(1, l2);
            shown = left;
        }
        pattern = ((elapsed / 500) & 1) ? 0xF0 : 0x0F;  /* alternate blink */
        if (pattern != last_pattern) {
            leds(pattern);
            last_pattern = pattern;
        }
    }
    leds(0x00);
    message("LOCK RELEASED", "TRY AGAIN", 1500);
}

static void wrong_pin(unsigned char id)
{
    char l2[17];

    wrong_tries++;
    log_add(EV_DENIED, id);

    if (wrong_tries >= MAX_TRIES) {
        lcd_show("ACCESS DENIED", "TOO MANY TRIES");
        flash_leds(3);
        lockout(id);
        return;
    }
    str_put(l2, "TRIES LEFT: ");
    l2[12] = (char)('0' + (MAX_TRIES - wrong_tries));
    l2[13] = '\0';
    lcd_show("ACCESS DENIED", l2);
    flash_leds(3);
    delay_ms(600);
}

/* ----------------------------- admin menu -------------------------------- */
static void admin_menu(void)
{
    unsigned char k, id;
    char l2[17];

    while (1) {
        lcd_show("1:LOG  2:RST PIN", "3:CLR LOG F:EXIT");
        k = get_key(IDLE_TIMEOUT_MS);
        if (k == KEY_NONE) {
            message("TIMED OUT", "LOGGED OUT", 1500);
            return;
        }
        if (k == KEY_CANCEL)
            return;

        if (k == 1) {
            if (view_log() == IN_TIMEOUT) {
                message("TIMED OUT", "LOGGED OUT", 1500);
                return;
            }
        } else if (k == 2) {
            unsigned char r = read_id("RESET WHICH PIN?", 1, &id);
            if (r == IN_TIMEOUT) {
                message("TIMED OUT", "LOGGED OUT", 1500);
                return;
            }
            if (r == IN_OK) {
                str_put(pins[id], default_pins[id]);
                log_add(EV_PIN_RST, id);
                id_name(l2, id);
                str_put(l2 + 5, " PIN RESET");
                message(l2, "TO DEFAULT", 1500);
            }
        } else if (k == 3) {
            log_head  = 0;
            log_count = 0;
            message("LOG CLEARED", "", 1500);
        }
    }
}

/* ----------------------------- main actions ------------------------------ */
static void login(unsigned char id)
{
    char pin[PIN_LEN + 1];
    char prompt[17];

    if (id == 0) {
        str_put(prompt, "ADMIN PIN");
    } else {
        id_name(prompt, id);            /* "USER2"                        */
        str_put(prompt + 5, " PIN");
    }
    if (aborted(read_pin(prompt, pin)))
        return;

    if (pin_equal(pin, pins[id])) {
        access_granted(id);
        if (id == 0)
            admin_menu();
    } else {
        wrong_pin(id);
    }
}

static void change_pin(void)
{
    unsigned char id;
    char old_pin[PIN_LEN + 1], new_pin[PIN_LEN + 1], confirm[PIN_LEN + 1];
    char l2[17];

    if (aborted(read_id("CHANGE PIN", 0, &id)))
        return;
    if (aborted(read_pin("ENTER OLD PIN", old_pin)))
        return;
    if (!pin_equal(old_pin, pins[id])) {
        wrong_pin(id);                  /* counts towards the lockout     */
        return;
    }
    wrong_tries = 0;
    lock_level  = 0;

    if (aborted(read_pin("ENTER NEW PIN", new_pin)))
        return;
    if (aborted(read_pin("CONFIRM NEW PIN", confirm)))
        return;
    if (!pin_equal(new_pin, confirm)) {
        message("PINS DONT MATCH", "PIN NOT CHANGED", 2000);
        return;
    }
    str_put(pins[id], new_pin);
    log_add(EV_PIN_CHG, id);
    id_name(l2, id);
    message("PIN CHANGED FOR", l2, 2000);
}

#if KEY_TEST_MODE
static void key_test(void)
{
    char l2[17];
    unsigned char k;

    lcd_show("KEY TEST MODE", "PRESS A KEY");
    while (1) {
        k = get_key(0);
        str_put(l2, "KEY CODE: ");
        l2[10] = (char)(k < 10 ? '0' + k : 'A' + k - 10);
        l2[11] = '\0';
        lcd_line(1, l2);
    }
}
#endif

/* ----------------------------- main -------------------------------------- */
int main(void)
{
    unsigned char k;

    SystemInit();
    SystemCoreClockUpdate();
    gpio_init();
    timer0_init();
    lcd_init();
    leds(0x00);

#if KEY_TEST_MODE
    key_test();
#endif

    message("ACCESS CONTROL", "SYSTEM READY", 1500);

    while (1) {
        lcd_show("ENTER ID (0-3)", "C: CHANGE PIN");
        k = get_key(0);                 /* home screen waits forever      */

        if (k < NUM_IDS)
            login(k);
        else if (k == KEY_CHANGE)
            change_pin();
        else if (k <= 9)
            message("INVALID ID", "USE 0 TO 3", 1500);
    }
}
