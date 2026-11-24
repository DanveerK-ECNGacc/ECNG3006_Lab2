#include <sam.h>

#include <FreeRTOS_SAMD21.h>


// =====================================================================

// Question 4: tracking BLOCKED tasks with LOGGING (single file, no config

// changes needed)

//

// Run it twice, changing ONLY the three priority numbers below:

// Case 1, all priorities the SAME : 1 / 1 / 1

// Case 2, all priorities DISTINCT : 3 / 2 / 1

// (leave configUSE_PREEMPTION at its default, 1, for both)

//

// Task 1 = LED ON (active wait 0.5 s while holding the lock)

// Task 2 = LED OFF (needs the lock only for the pin write)

// Task 3 = holds the lock for a slow operation, then prints the report

//

// All three run once every 2 s, started at staggered times:

// T3 at 0 ms, T2 at 50 ms, T1 at 100 ms

//

// Mechanism: DEFERRED EVENT LOGGING.

// take_logged() replaces xSemaphoreTake(). It first tries the mutex with

// a timeout of 0; if that fails the task is about to block, so the block

// is counted, timed and written into a small RAM log. Nothing is printed

// from inside T1/T2 (UART printing takes ~100 us per character and would

// distort the timing); T3 prints the log afterwards.

//

// Printed by T3 once per cycle (1 tick = 1 ms):

// T1 latency = time from when T1 was DUE to run until it got the lock

// T2 response = time from when T2 was DUE until its LED-OFF write

// blocked = per task: how many times it blocked on the mutex, the total

// and longest time spent blocked

// log = every individual block since the last report

// logging cost= CPU cycles spent inside the logging code itself

// =====================================================================


// ---------- Task priorities (higher number = higher priority) ----------

// EDIT THESE THREE LINES for the two cases:

// Case 1 (all same): 1 / 1 / 1

// Case 2 (all distinct): 3 / 2 / 1

static const int PRIO_TASK1 = 1;

static const int PRIO_TASK2 = 1;

static const int PRIO_TASK3 = 1;


static const int TASK_STACK_WORDS = configMINIMAL_STACK_SIZE;


// ---------- Experiment timing (ms) ----------

#define PERIOD_MS 2000 // every task repeats every 2 s

#define T2_OFFSET_MS 50 // T2 first due at 50 ms

#define T1_OFFSET_MS 100 // T1 first due at 100 ms

#define T1_ON_MS 500 // T1: LED ON, active wait (wall-clock)

#define T2_WORK_MS 400 // T2: CPU-bound work, needs no lock

#define T3_HOLD_MS 300 // T3: CPU-bound work while HOLDING the lock

#define CAL_MS 25 // CPU calibration window

#define REPORT_AT_MS 1600 // T3 prints this far into each cycle


// ---------- Lock that guards the shared LED pin (PB08) ----------

static SemaphoreHandle_t ledMutex;


// ---------- Measurements (1 tick = 1 ms at 1000 Hz) ----------

static volatile uint32_t t1Latency = 0;

static volatile uint32_t t2Response = 0;

static volatile uint32_t t1OnAt = 0; // tick of T1's LED-ON write

static volatile uint32_t t2OffAt = 0; // tick of T2's LED-OFF write


// =====================================================================

// Q4: the event log and the blocked-task statistics

// =====================================================================

#define LOG_LEN 16


struct LogEntry {

uint32_t tick; // tick when the task got the mutex and resumed

uint32_t waited; // how long it was blocked (ms)

uint8_t task; // 0 = Task1, 1 = Task2, 2 = Task3

};


static LogEntry logBuf[LOG_LEN];

static volatile uint32_t logWrite = 0; // total entries ever written

static uint32_t logRead = 0; // entries already printed (T3 only)


static volatile uint32_t blkCount[3]; // times the task blocked on the mutex

static volatile uint32_t blkTotal[3]; // total ms spent blocked

static volatile uint32_t blkMax[3]; // longest single block (ms)


// Cost of the logging itself, in CPU cycles, measured with SysTick->VAL

static volatile uint32_t logCalls = 0;

static volatile uint32_t logCycles = 0;

static volatile uint32_t logCyclesMax = 0;


// Takes the mutex. If the task has to wait for it, the block is logged.

static void take_logged(int id) {

TickType_t t0 = xTaskGetTickCount();


if (xSemaphoreTake(ledMutex, 0) == pdTRUE) {

return; // free: no block, nothing to log

}


// The mutex is held by someone else: this task WILL block now

xSemaphoreTake(ledMutex, portMAX_DELAY);

TickType_t waited = xTaskGetTickCount() - t0;


// ---- the logging itself (its cycle cost is measured) ----

uint32_t c0 = SysTick->VAL;


blkCount[id] = blkCount[id] + 1;

blkTotal[id] = blkTotal[id] + waited;

if (waited > blkMax[id]) blkMax[id] = waited;


taskENTER_CRITICAL(); // the log is shared by all tasks

LogEntry &e = logBuf[logWrite % LOG_LEN];

e.tick = xTaskGetTickCount();

e.waited = waited;

e.task = (uint8_t)id;

logWrite = logWrite + 1;

taskEXIT_CRITICAL();


uint32_t c1 = SysTick->VAL; // SysTick counts DOWN

uint32_t load = SysTick->LOAD;

uint32_t cost = (c0 >= c1) ? (c0 - c1) : (c0 + load + 1u - c1);

logCalls = logCalls + 1;

logCycles = logCycles + cost;

if (cost > logCyclesMax) logCyclesMax = cost;

}


// ---------- CPU-time busy work ----------

// burn_ms() uses up CPU TIME, not wall-clock time: if the task is preempted

// it still has the same amount of work left when it resumes. (A tick-based

// wait would "finish" while preempted and break the experiment.)

static volatile uint32_t calChunks = 0; // 1000-iteration chunks done in CAL_MS


static void burn_chunks(uint32_t n) {

while (n--) {

for (volatile uint32_t i = 0; i < 1000; i++) { }

}

}


static void calibrate(void) {

TickType_t t = xTaskGetTickCount();

while (xTaskGetTickCount() == t) { } // align to a tick edge

TickType_t start = xTaskGetTickCount();

uint32_t n = 0;

while ((xTaskGetTickCount() - start) < CAL_MS) {

burn_chunks(1);

n++;

}

calChunks = n;

}


static void burn_ms(uint32_t ms) {

burn_chunks((calChunks * ms) / CAL_MS);

}


// Active wait on the wall clock (this is the "actively wait 0.5 s" of the spec)

static void spin_ms(uint32_t ms) {

TickType_t start = xTaskGetTickCount();

while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(ms)) {

// spin

}

}


// ---------- Raw SERCOM5 UART (MKR Zero: pin 14 = TX/PB22, pin 13 = RX/PB23) ----------

static void uart_init(void) {

// Route PB22/PB23 to SERCOM5 (peripheral function D = 3)

PORT->Group[1].PINCFG[22].reg |= PORT_PINCFG_PMUXEN;

PORT->Group[1].PINCFG[23].reg |= PORT_PINCFG_PMUXEN;

PORT->Group[1].PMUX[11].reg = PORT_PMUX_PMUXE(3) | PORT_PMUX_PMUXO(3);


// Power and clock SERCOM5 from GCLK0 (48 MHz)

PM->APBCMASK.reg |= PM_APBCMASK_SERCOM5;

GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_SERCOM5_CORE;

while (GCLK->STATUS.bit.SYNCBUSY);


// Reset, then configure: internal clock, TX on PAD2, RX on PAD3, LSB first

SERCOM5->USART.CTRLA.reg = SERCOM_USART_CTRLA_SWRST;

while (SERCOM5->USART.SYNCBUSY.bit.SWRST);


SERCOM5->USART.CTRLA.reg = SERCOM_USART_CTRLA_DORD |

SERCOM_USART_CTRLA_MODE(1) |

SERCOM_USART_CTRLA_RXPO(3) |

SERCOM_USART_CTRLA_TXPO(1);

SERCOM5->USART.CTRLB.reg = SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_RXEN;

while (SERCOM5->USART.SYNCBUSY.bit.CTRLB);


SERCOM5->USART.BAUD.reg = 63019; // 115200 baud @ 48 MHz

SERCOM5->USART.CTRLA.bit.ENABLE = 1;

while (SERCOM5->USART.SYNCBUSY.bit.ENABLE);

}


static void uart_puts(const char *s) {

while (*s) {

while (!SERCOM5->USART.INTFLAG.bit.DRE);

SERCOM5->USART.DATA.reg = (uint8_t)*s++;

}

}


static void uart_putu(uint32_t v) {

char buf[11];

int i = 10;

buf[i] = '\0';

do { buf[--i] = '0' + (v % 10); v /= 10; } while (v);

uart_puts(&buf[i]);

}


static const char *task_name(int i) {

return (i == 0) ? "Task1" : ((i == 1) ? "Task2" : "Task3");

}


// Prints the cycle results, the blocked statistics and every new log entry

static void print_report(uint32_t cycle) {

uart_puts("cycle ");

uart_putu(cycle);

uart_puts(": T1 latency=");

uart_putu(t1Latency);

uart_puts(" ms | T2 response=");

uart_putu(t2Response);

uart_puts(" ms | LED ON@");

uart_putu(t1OnAt);

uart_puts(" OFF@");

uart_putu(t2OffAt);

uart_puts("\r\n");


// Cumulative since boot: count / total ms / longest ms

uart_puts(" blocked on mutex (n/total/max ms):");

for (int i = 0; i < 3; i++) {

uart_puts(i == 0 ? " T1 " : " | T");

if (i != 0) { uart_putu((uint32_t)(i + 1)); uart_puts(" "); }

uart_putu(blkCount[i]);

uart_puts("/");

uart_putu(blkTotal[i]);

uart_puts("/");

uart_putu(blkMax[i]);

}

uart_puts("\r\n");


// Every individual block since the last report

if (logWrite - logRead > LOG_LEN) {

logRead = logWrite - LOG_LEN; // log wrapped: oldest entries lost

}

while (logRead < logWrite) {

LogEntry e = logBuf[logRead % LOG_LEN];

uart_puts(" log: ");

uart_puts(task_name(e.task));

uart_puts(" blocked ");

uart_putu(e.waited);

uart_puts(" ms (resumed at tick ");

uart_putu(e.tick);

uart_puts(")\r\n");

logRead = logRead + 1;

}


uart_puts(" logging cost: calls=");

uart_putu(logCalls);

uart_puts(" avg=");

uart_putu(logCalls ? (logCycles / logCalls) : 0);

uart_puts(" cycles max=");

uart_putu(logCyclesMax);

uart_puts(" cycles\r\n");

}



// Task 1: LED ON, actively wait 0.5 s while holding the lock, then yield

void vTask1(void *pvParameters) {

TickType_t lastWake = 0;

vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(T1_OFFSET_MS));


while (1) {

// lastWake is the tick this cycle was DUE

take_logged(0); // logs it if T1 has to block

TickType_t got = xTaskGetTickCount();

t1Latency = got - lastWake; // how late we got the lock

t1OnAt = got;

PORT->Group[1].OUTSET.reg = PORT_PB08; // LED ON


spin_ms(T1_ON_MS); // active wait, lock held


xSemaphoreGive(ledMutex);


taskYIELD();

vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PERIOD_MS));

}

}


// Task 2: CPU-bound work (no lock), then LED OFF

void vTask2(void *pvParameters) {

TickType_t lastWake = 0;

vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(T2_OFFSET_MS));


while (1) {

burn_ms(T2_WORK_MS); // needs no lock


take_logged(1); // logs it if T2 has to block

TickType_t now = xTaskGetTickCount();

t2OffAt = now;

t2Response = now - lastWake; // how late the OFF happened

PORT->Group[1].OUTCLR.reg = PORT_PB08; // LED OFF

xSemaphoreGive(ledMutex);


vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PERIOD_MS));

}

}


// Task 3: holds the lock for a slow operation, then prints the report

void vTask3(void *pvParameters) {

uint32_t cycle = 0;


calibrate(); // done before T2/T1 wake up


uart_puts("Q4 logging | priorities T1=");

uart_putu((uint32_t)PRIO_TASK1);

uart_puts(" T2=");

uart_putu((uint32_t)PRIO_TASK2);

uart_puts(" T3=");

uart_putu((uint32_t)PRIO_TASK3);

uart_puts(" | preempt=");

uart_putu((uint32_t)configUSE_PREEMPTION);

uart_puts(" | tick_hz=");

uart_putu((uint32_t)configTICK_RATE_HZ);

uart_puts(" | calChunks=");

uart_putu(calChunks);

uart_puts("\r\n");


TickType_t lastWake = 0;


while (1) {

// Slow work while owning the shared resource

take_logged(2);

burn_ms(T3_HOLD_MS);

xSemaphoreGive(ledMutex);


// Sleep until 1.6 s into the cycle. T1 and T2 have finished by then,

// so this cycle's numbers are complete, and the (slow) printing cannot

// disturb what is being measured.

vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(REPORT_AT_MS));

print_report(cycle);

cycle++;


// Sleep until the start of the next cycle

vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PERIOD_MS - REPORT_AT_MS));

}

}


int main(void) {

// Initialize standard microcontroller system clocks (48MHz layout)

SystemInit();

uart_init();


// Configure Pin PB08 (Built-in LED) as a digital OUTPUT

PORT->Group[1].DIRSET.reg = PORT_PB08;


// Create the lock that guards the shared LED pin

// Real mutex: priority inheritance is built in

ledMutex = xSemaphoreCreateMutex();

configASSERT(ledMutex != NULL);


// Create the three tasks, each with its own priority constant

xTaskCreate(vTask1, "Task1", TASK_STACK_WORDS, NULL, PRIO_TASK1, NULL);

xTaskCreate(vTask2, "Task2", TASK_STACK_WORDS, NULL, PRIO_TASK2, NULL);

xTaskCreate(vTask3, "Task3", TASK_STACK_WORDS, NULL, PRIO_TASK3, NULL);


// Boot the FreeRTOS Scheduler engine execution path

vTaskStartScheduler();


while (1) {

// Safety lock trap loop

__NOP();

}

return 0;

}


// Retain underlying framework linkages to keep compilation targets happy

extern "C" {

void setup() {

// Left completely blank

}

void loop() {

// Left completely blank

}

}