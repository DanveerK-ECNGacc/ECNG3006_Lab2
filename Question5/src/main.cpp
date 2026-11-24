#include <sam.h>
#include <FreeRTOS_SAMD21.h>

// =====================================================================
// Question 5: replacing the MUTEX with a MESSAGE QUEUE (or a SEMAPHORE)
// Based on the Q4 program (same tasks, same timing, same report).
//
// IPC_MODE selects how the shared LED pin (PB08) is protected:
//
//   IPC_MUTEX  (0)  : the Q4 version, unchanged. xSemaphoreCreateMutex().
//                     Priority inheritance is built in.
//   IPC_BINSEM (1)  : a BINARY SEMAPHORE used as the lock.
//                     xSemaphoreCreateBinary() + one give. Same code as
//                     the mutex, but NO priority inheritance.
//   IPC_QUEUE  (2)  : a MESSAGE QUEUE + a "gatekeeper" task (default).
//                     Only the gatekeeper ever touches PB08. T1 and T2
//                     send it an ON / OFF message and carry on; nobody
//                     ever holds a lock, so nobody waits for one.
//
// Run it in at least two modes (MUTEX and QUEUE) and both priority cases:
//   Case 1, all priorities the SAME   : 1 / 1 / 1
//   Case 2, all priorities DISTINCT   : 3 / 2 / 1
// (leave configUSE_PREEMPTION at its default, 1)
//
// Task 1 = LED ON, then actively waits 0.5 s
//          (lock modes: while holding the lock)
// Task 2 = CPU work (400 ms), then LED OFF
// Task 3 = slow operation (300 ms of CPU work), then prints the report
//          (lock modes: holds the lock for the slow operation;
//           queue mode: needs no lock, the LED belongs to the gatekeeper)
// Gate   = queue mode only: receives LED messages and writes PB08
//
// All three run once every 2 s, started at staggered times:
//   T3 at 0 ms, T2 at 50 ms, T1 at 100 ms
//
// Printed by T3 once per cycle (1 tick = 1 ms):
//   T1 latency  = time from when T1 was DUE until the LED actually went ON
//   T2 response = time from when T2 was DUE until the LED actually went OFF
//   blocked     = per task: how many times it BLOCKED in the IPC call
//                 (lock modes: lock already held; queue mode: queue full)
//   log         = every individual block since the last report
//   IPC cost    = CPU cycles for one uncontended LED access, measured at
//                 boot: take+give (lock modes) or one send including the
//                 switch to the gatekeeper and back (queue mode)
//
// The file compiles as C (main.c) or C++ (main.cpp).
// =====================================================================

// ---------- IPC selection: change this line, rebuild, upload ----------
#define IPC_MUTEX  0
#define IPC_BINSEM 1
#define IPC_QUEUE  2

#ifndef IPC_MODE            // can also be set from platformio.ini build_flags
#define IPC_MODE IPC_QUEUE //Switch between IPC_MUTEX and IPC_QUEUE
#endif

// ---------- Task priorities (higher number = higher priority) ----------
// EDIT THESE THREE LINES for the two cases:
//   Case 1 (all same):     1 / 1 / 1
//   Case 2 (all distinct): 3 / 2 / 1
static const int PRIO_TASK1 = 3;
static const int PRIO_TASK2 = 2;
static const int PRIO_TASK3 = 1;
#if (IPC_MODE == IPC_QUEUE)
static const int PRIO_GATE  = 4;   // above every application task, so a
                                   // message is applied as soon as it is sent
#endif

static const int TASK_STACK_WORDS = configMINIMAL_STACK_SIZE;

// ---------- Experiment timing (ms) ----------
#define PERIOD_MS     2000   // every task repeats every 2 s
#define T2_OFFSET_MS  50     // T2 first due at 50 ms
#define T1_OFFSET_MS  100    // T1 first due at 100 ms
#define T1_ON_MS      500    // T1: LED ON, active wait (wall-clock)
#define T2_WORK_MS    400    // T2: CPU-bound work, needs no lock
#define T3_HOLD_MS    300    // T3: CPU-bound slow operation
#define CAL_MS        25     // CPU calibration window
#define REPORT_AT_MS  1600   // T3 prints this far into each cycle

#define QUEUE_LEN     4      // LED messages the queue can hold
#define BENCH_N       100    // repetitions of the IPC cost measurement

// ---------- The inter-task communication object ----------
#if (IPC_MODE == IPC_QUEUE)
#define LED_OFF 0
#define LED_ON  1

typedef struct {
    uint8_t    cmd;          // LED_ON or LED_OFF
    uint8_t    sender;       // 0 = Task1, 1 = Task2, 2 = benchmark
    TickType_t due;          // tick the sender's cycle was due
} LedMsg;

static QueueHandle_t ledQueue;       // messages for the gatekeeper
#else
static SemaphoreHandle_t ledLock;    // mutex or binary semaphore
#endif

// ---------- Measurements (1 tick = 1 ms at 1000 Hz) ----------
static volatile uint32_t t1Latency  = 0;
static volatile uint32_t t2Response = 0;
static volatile uint32_t t1OnAt     = 0;   // tick of the LED-ON write
static volatile uint32_t t2OffAt    = 0;   // tick of the LED-OFF write

// =====================================================================
// Event log and blocked-task statistics (unchanged from Q4)
// =====================================================================
#define LOG_LEN 16

typedef struct {
    uint32_t tick;      // tick when the task got through and resumed
    uint32_t waited;    // how long it was blocked (ms)
    uint8_t  task;      // 0 = Task1, 1 = Task2, 2 = Task3
} LogEntry;

static LogEntry          logBuf[LOG_LEN];
static volatile uint32_t logWrite = 0;     // total entries ever written
static uint32_t          logRead  = 0;     // entries already printed (T3)

static volatile uint32_t blkCount[3];      // times the task blocked
static volatile uint32_t blkTotal[3];      // total ms spent blocked
static volatile uint32_t blkMax[3];        // longest single block (ms)

// Cost of the logging itself, in CPU cycles, measured with SysTick->VAL
static volatile uint32_t logCalls     = 0;
static volatile uint32_t logCycles    = 0;
static volatile uint32_t logCyclesMax = 0;

// Cost of one uncontended IPC access (measured once at boot)
static volatile uint32_t benchMin = 0;
static volatile uint32_t benchAvg = 0;
static volatile uint32_t benchMax = 0;

// CPU cycles between two SysTick->VAL readings (SysTick counts DOWN,
// reloads from LOAD every 1 ms; valid for intervals shorter than 1 ms)
static uint32_t cycles_between(uint32_t c0, uint32_t c1) {
    uint32_t load = SysTick->LOAD;
    return (c0 >= c1) ? (c0 - c1) : (c0 + load + 1u - c1);
}

// Records one block in the statistics and the log (called after the block)
static void log_block(int id, TickType_t waited) {
    uint32_t c0 = SysTick->VAL;

    blkCount[id] = blkCount[id] + 1;
    blkTotal[id] = blkTotal[id] + waited;
    if (waited > blkMax[id]) blkMax[id] = waited;

    taskENTER_CRITICAL();                  // the log is shared by all tasks
    LogEntry *e = &logBuf[logWrite % LOG_LEN];
    e->tick   = xTaskGetTickCount();
    e->waited = waited;
    e->task   = (uint8_t)id;
    logWrite  = logWrite + 1;
    taskEXIT_CRITICAL();

    uint32_t cost = cycles_between(c0, SysTick->VAL);
    logCalls  = logCalls + 1;
    logCycles = logCycles + cost;
    if (cost > logCyclesMax) logCyclesMax = cost;
}

// =====================================================================
// The IPC layer: the ONLY code that differs between the modes
// =====================================================================
#if (IPC_MODE == IPC_QUEUE)

// Sends an LED command to the gatekeeper. Never waits unless the queue is
// full; if it is, the block is logged exactly like a mutex block in Q4.
static void led_send(int id, uint8_t cmd, TickType_t due) {
    LedMsg m;
    m.cmd    = cmd;
    m.sender = (uint8_t)id;
    m.due    = due;

    TickType_t t0 = xTaskGetTickCount();

    if (xQueueSend(ledQueue, &m, 0) == pdTRUE) {
        return;                            // space in the queue: no block
    }

    // Queue full: this task WILL block now
    xQueueSend(ledQueue, &m, portMAX_DELAY);
    log_block(id, xTaskGetTickCount() - t0);
}

// Gatekeeper: the only task that writes PB08
static void vLedGatekeeper(void *pvParameters) {
    (void)pvParameters;
    LedMsg m;

    while (1) {
        if (xQueueReceive(ledQueue, &m, portMAX_DELAY) == pdTRUE) {
            TickType_t now = xTaskGetTickCount();

            if (m.cmd == LED_ON) {
                PORT->Group[1].OUTSET.reg = PORT_PB08;     // LED ON
            } else {
                PORT->Group[1].OUTCLR.reg = PORT_PB08;     // LED OFF
            }

            if (m.sender == 0) {                           // from Task 1
                t1OnAt    = now;
                t1Latency = now - m.due;
            } else if (m.sender == 1) {                    // from Task 2
                t2OffAt    = now;
                t2Response = now - m.due;
            }                                              // 2 = benchmark
        }
    }
}

#else  // IPC_MUTEX or IPC_BINSEM

// Takes the lock. If the task has to wait for it, the block is logged.
// (identical to take_logged() in Q4)
static void lock_take(int id) {
    TickType_t t0 = xTaskGetTickCount();

    if (xSemaphoreTake(ledLock, 0) == pdTRUE) {
        return;                            // free: no block, nothing to log
    }

    // The lock is held by someone else: this task WILL block now
    xSemaphoreTake(ledLock, portMAX_DELAY);
    log_block(id, xTaskGetTickCount() - t0);
}

static void lock_give(void) {
    xSemaphoreGive(ledLock);
}

#endif

static void ipc_create(void) {
#if (IPC_MODE == IPC_MUTEX)
    ledLock = xSemaphoreCreateMutex();         // starts available
    configASSERT(ledLock != NULL);
#elif (IPC_MODE == IPC_BINSEM)
    ledLock = xSemaphoreCreateBinary();        // starts EMPTY (taken)
    configASSERT(ledLock != NULL);
    xSemaphoreGive(ledLock);                   // make it available once
#else
    ledQueue = xQueueCreate(QUEUE_LEN, sizeof(LedMsg));
    configASSERT(ledQueue != NULL);
#endif
}

// Measures one uncontended LED access BENCH_N times (called by T3 at boot,
// before T1 and T2 wake up, so nothing competes with it)
static void ipc_benchmark(void) {
    uint32_t sum = 0;
    uint32_t mn  = 0xFFFFFFFFu;
    uint32_t mx  = 0;

    for (int i = 0; i < BENCH_N; i++) {
#if (IPC_MODE == IPC_QUEUE)
        LedMsg m;
        m.cmd    = LED_OFF;                    // LED is off anyway
        m.sender = 2;
        m.due    = 0;
        uint32_t c0 = SysTick->VAL;
        xQueueSend(ledQueue, &m, 0);           // gatekeeper runs inside this
        uint32_t c1 = SysTick->VAL;
#else
        uint32_t c0 = SysTick->VAL;
        xSemaphoreTake(ledLock, 0);
        xSemaphoreGive(ledLock);
        uint32_t c1 = SysTick->VAL;
#endif
        uint32_t c = cycles_between(c0, c1);
        sum += c;
        if (c < mn) mn = c;
        if (c > mx) mx = c;
    }
    benchMin = mn;
    benchAvg = sum / BENCH_N;
    benchMax = mx;
}

// ---------- CPU-time busy work (unchanged from Q4) ----------
// burn_ms() uses up CPU TIME, not wall-clock time: if the task is preempted
// it still has the same amount of work left when it resumes.
static volatile uint32_t calChunks = 0;  // 1000-iteration chunks in CAL_MS

static void burn_chunks(uint32_t n) {
    while (n--) {
        for (volatile uint32_t i = 0; i < 1000; i++) { }
    }
}

static void calibrate(void) {
    TickType_t t = xTaskGetTickCount();
    while (xTaskGetTickCount() == t) { }       // align to a tick edge
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

// Active wait on the wall clock (the "actively wait 0.5 s" of the spec)
static void spin_ms(uint32_t ms) {
    TickType_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(ms)) {
        // spin
    }
}

// ---------- Raw SERCOM5 UART (pin 14 = TX/PB22, pin 13 = RX/PB23) ----------
static void uart_init(void) {
    // Route PB22/PB23 to SERCOM5 (peripheral function D = 3)
    PORT->Group[1].PINCFG[22].reg |= PORT_PINCFG_PMUXEN;
    PORT->Group[1].PINCFG[23].reg |= PORT_PINCFG_PMUXEN;
    PORT->Group[1].PMUX[11].reg = PORT_PMUX_PMUXE(3) | PORT_PMUX_PMUXO(3);

    // Power and clock SERCOM5 from GCLK0 (48 MHz)
    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM5;
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 |
                        GCLK_CLKCTRL_ID_SERCOM5_CORE;
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

    SERCOM5->USART.BAUD.reg = 63019;           // 115200 baud @ 48 MHz
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

// Prints CPU cycles at 48 MHz as microseconds with one decimal
static void uart_putus(uint32_t cycles) {
    uint32_t x10 = (cycles * 10u) / 48u;
    uart_putu(x10 / 10u);
    uart_puts(".");
    uart_putu(x10 % 10u);
}

static const char *task_name(int i) {
    return (i == 0) ? "Task1" : ((i == 1) ? "Task2" : "Task3");
}

static const char *ipc_name(void) {
#if (IPC_MODE == IPC_MUTEX)
    return "MUTEX";
#elif (IPC_MODE == IPC_BINSEM)
    return "BINARY SEMAPHORE";
#else
    return "MESSAGE QUEUE";
#endif
}

// Prints the cycle results, the blocked statistics and every new log entry
static void print_report(uint32_t cycle) {
    uart_puts("cycle ");
    uart_putu(cycle);
    uart_puts(" [");
    uart_puts(ipc_name());
    uart_puts("]: T1 latency=");
    uart_putu(t1Latency);
    uart_puts(" ms | T2 response=");
    uart_putu(t2Response);
    uart_puts(" ms | LED ON@");
    uart_putu(t1OnAt);
    uart_puts(" OFF@");
    uart_putu(t2OffAt);
    uart_puts("\r\n");

    // Cumulative since boot: count / total ms / longest ms
    uart_puts(" blocked in IPC (n/total/max ms):");
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
        logRead = logWrite - LOG_LEN;          // log wrapped: oldest lost
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

    uart_puts(" IPC cost per LED access (uncontended, n=");
    uart_putu(BENCH_N);
    uart_puts("): min/avg/max=");
    uart_putu(benchMin);
    uart_puts("/");
    uart_putu(benchAvg);
    uart_puts("/");
    uart_putu(benchMax);
    uart_puts(" cycles (avg ");
    uart_putus(benchAvg);
    uart_puts(" us)\r\n");
}

// Task 1: LED ON, then actively wait 0.5 s
void vTask1(void *pvParameters) {
    (void)pvParameters;
    TickType_t lastWake = 0;
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(T1_OFFSET_MS));

    while (1) {
        // lastWake is the tick this cycle was DUE
#if (IPC_MODE == IPC_QUEUE)
        led_send(0, LED_ON, lastWake);         // gatekeeper switches it ON
        spin_ms(T1_ON_MS);                     // active wait, holds nothing
#else
        lock_take(0);                          // logs it if T1 has to block
        TickType_t got = xTaskGetTickCount();
        t1Latency = got - lastWake;            // how late the LED went ON
        t1OnAt    = got;
        PORT->Group[1].OUTSET.reg = PORT_PB08; // LED ON
        spin_ms(T1_ON_MS);                     // active wait, lock held
        lock_give();
#endif
        taskYIELD();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PERIOD_MS));
    }
}

// Task 2: CPU-bound work (no lock), then LED OFF
void vTask2(void *pvParameters) {
    (void)pvParameters;
    TickType_t lastWake = 0;
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(T2_OFFSET_MS));

    while (1) {
        burn_ms(T2_WORK_MS);                   // needs no lock
#if (IPC_MODE == IPC_QUEUE)
        led_send(1, LED_OFF, lastWake);        // gatekeeper switches it OFF
#else
        lock_take(1);                          // logs it if T2 has to block
        TickType_t now = xTaskGetTickCount();
        t2OffAt    = now;
        t2Response = now - lastWake;           // how late the OFF happened
        PORT->Group[1].OUTCLR.reg = PORT_PB08; // LED OFF
        lock_give();
#endif
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PERIOD_MS));
    }
}

// Task 3: slow operation, then prints the report
void vTask3(void *pvParameters) {
    (void)pvParameters;
    uint32_t cycle = 0;

    calibrate();                               // done before T2/T1 wake up
    ipc_benchmark();                           // ~1 ms, still before T2/T1

    uart_puts("Q5 IPC=");
    uart_puts(ipc_name());
    uart_puts(" | priorities T1=");
    uart_putu((uint32_t)PRIO_TASK1);
    uart_puts(" T2=");
    uart_putu((uint32_t)PRIO_TASK2);
    uart_puts(" T3=");
    uart_putu((uint32_t)PRIO_TASK3);
    uart_puts(" | preempt=");
    uart_putu((uint32_t)configUSE_PREEMPTION);
    uart_puts(" | calChunks=");
    uart_putu(calChunks);
    uart_puts("\r\n");

    TickType_t lastWake = 0;

    while (1) {
#if (IPC_MODE == IPC_QUEUE)
        burn_ms(T3_HOLD_MS);                   // no lock to hold any more
#else
        lock_take(2);                          // slow work while owning
        burn_ms(T3_HOLD_MS);                   // the shared resource
        lock_give();
#endif

        // Sleep until 1.6 s into the cycle. T1 and T2 have finished by then,
        // so this cycle's numbers are complete, and the (slow) printing
        // cannot disturb what is being measured.
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(REPORT_AT_MS));
        print_report(cycle);
        cycle++;

        // Sleep until the start of the next cycle
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PERIOD_MS - REPORT_AT_MS));
    }
}

int main(void) {
    // Initialize standard microcontroller system clocks (48 MHz layout)
    SystemInit();
    uart_init();

    // Configure Pin PB08 (Built-in LED) as a digital OUTPUT, start OFF
    PORT->Group[1].DIRSET.reg = PORT_PB08;
    PORT->Group[1].OUTCLR.reg = PORT_PB08;

    // Create the mutex, the binary semaphore or the queue (IPC_MODE)
    ipc_create();

    // Create the tasks, each with its own priority constant
    xTaskCreate(vTask1, "Task1", TASK_STACK_WORDS, NULL, PRIO_TASK1, NULL);
    xTaskCreate(vTask2, "Task2", TASK_STACK_WORDS, NULL, PRIO_TASK2, NULL);
    xTaskCreate(vTask3, "Task3", TASK_STACK_WORDS, NULL, PRIO_TASK3, NULL);
#if (IPC_MODE == IPC_QUEUE)
    xTaskCreate(vLedGatekeeper, "Gate", TASK_STACK_WORDS, NULL, PRIO_GATE, NULL);
#endif

    // Start the FreeRTOS scheduler (does not return)
    vTaskStartScheduler();

    while (1) {
        __NOP();                               // only if the heap is too small
    }
    return 0;
}

// Retain underlying framework linkages to keep compilation targets happy
#ifdef __cplusplus
extern "C" {
#endif
void setup(void) { }
void loop(void) { }
#ifdef __cplusplus
}
#endif
