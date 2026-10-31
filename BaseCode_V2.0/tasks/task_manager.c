#include "tasks/task_manager.h"
#include "tasks/motor_task.h"
#include "tasks/acquisition_task.h"
#include "tasks/estimation_task.h"
#include "tasks/communication_task.h"
#include "logger.h"

/* Queue Handles */
QueueHandle_t q_sensor_data = NULL;
QueueHandle_t q_robot_state = NULL;
QueueHandle_t q_motor_cmd   = NULL;
QueueHandle_t q_usb_rx      = NULL;

/* Synchronization Handles */
SemaphoreHandle_t xUsbTxSemaphore = NULL;
SemaphoreHandle_t xRobotStateMutex = NULL;

/* Static Queue Control Blocks & Storage */
static StaticQueue_t s_q_sensor_data_struct;
static uint8_t       s_q_sensor_data_storage[Q_SENSOR_DATA_LEN * sizeof(sensor_data_t)];

static StaticQueue_t s_q_robot_state_struct;
static uint8_t       s_q_robot_state_storage[Q_ROBOT_STATE_LEN * sizeof(robot_state_t)];

static StaticQueue_t s_q_motor_cmd_struct;
static uint8_t       s_q_motor_cmd_storage[Q_MOTOR_CMD_LEN * sizeof(motor_cmd_t)];

static StaticQueue_t s_q_usb_rx_struct;
static uint8_t       s_q_usb_rx_storage[Q_USB_RX_LEN * sizeof(uint8_t)];

/* Static Semaphore Control Blocks */
static StaticSemaphore_t s_usb_tx_sem_struct;
static StaticSemaphore_t s_state_mutex_struct;

/* Static Task Control Blocks & Stacks */
static StaticTask_t s_motor_task_tcb;
static StackType_t  s_motor_task_stack[MOTOR_TASK_STACK_SIZE];

static StaticTask_t s_acq_task_tcb;
static StackType_t  s_acq_task_stack[ACQUISITION_TASK_STACK_SIZE];

static StaticTask_t s_est_task_tcb;
static StackType_t  s_est_task_stack[ESTIMATION_TASK_STACK_SIZE];

static StaticTask_t s_comm_task_tcb;
static StackType_t  s_comm_task_stack[COMMUNICATION_TASK_STACK_SIZE];

/* Deadman timestamp variable */
static volatile uint32_t s_last_jetson_packet_tick = 0;

/* Protected Robot State Snapshot */
static robot_state_t s_global_state_snapshot = {0};

/* Idle & Timer Task Static Memory Required by configSUPPORT_STATIC_ALLOCATION */
static StaticTask_t s_idle_task_tcb;
static StackType_t  s_idle_task_stack[configMINIMAL_STACK_SIZE];

static StaticTask_t s_timer_task_tcb;
static StackType_t  s_timer_task_stack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t  **ppxIdleTaskStackBuffer,
                                   uint32_t      *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer   = &s_idle_task_tcb;
    *ppxIdleTaskStackBuffer = s_idle_task_stack;
    *pulIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t  **ppxTimerTaskStackBuffer,
                                    uint32_t      *pulTimerTaskStackSize)
{
    *ppxTimerTaskTCBBuffer   = &s_timer_task_tcb;
    *ppxTimerTaskStackBuffer = s_timer_task_stack;
    *pulTimerTaskStackSize   = configTIMER_TASK_STACK_DEPTH;
}

/* Stack Overflow Detection Hook (configCHECK_FOR_STACK_OVERFLOW = 2) */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    /* Trap stack overflow error safely */
    taskDISABLE_INTERRUPTS();
    for (;;);
}

void task_manager_notify_jetson_packet_received(void)
{
    s_last_jetson_packet_tick = xTaskGetTickCount();
}

uint32_t task_manager_get_last_jetson_packet_tick(void)
{
    return s_last_jetson_packet_tick;
}

void task_manager_set_state_snapshot(const robot_state_t *new_state)
{
    if (!new_state || xRobotStateMutex == NULL) return;
    if (xSemaphoreTake(xRobotStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        s_global_state_snapshot = *new_state;
        xSemaphoreGive(xRobotStateMutex);
    }
}

void task_manager_get_state_snapshot(robot_state_t *out_state)
{
    if (!out_state || xRobotStateMutex == NULL) return;
    if (xSemaphoreTake(xRobotStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        *out_state = s_global_state_snapshot;
        xSemaphoreGive(xRobotStateMutex);
    }
}

void task_manager_init(void)
{
    /* 1. Initialize Logger */
    logger_init();

    /* 2. Create Static Queues */
    q_sensor_data = xQueueCreateStatic(
        Q_SENSOR_DATA_LEN, sizeof(sensor_data_t),
        s_q_sensor_data_storage, &s_q_sensor_data_struct
    );

    q_robot_state = xQueueCreateStatic(
        Q_ROBOT_STATE_LEN, sizeof(robot_state_t),
        s_q_robot_state_storage, &s_q_robot_state_struct
    );

    q_motor_cmd = xQueueCreateStatic(
        Q_MOTOR_CMD_LEN, sizeof(motor_cmd_t),
        s_q_motor_cmd_storage, &s_q_motor_cmd_struct
    );

    q_usb_rx = xQueueCreateStatic(
        Q_USB_RX_LEN, sizeof(uint8_t),
        s_q_usb_rx_storage, &s_q_usb_rx_struct
    );

    /* 3. Create Static Semaphores & Mutexes */
    xUsbTxSemaphore = xSemaphoreCreateBinaryStatic(&s_usb_tx_sem_struct);
    if (xUsbTxSemaphore != NULL) {
        /* Initially USB transmitter is available */
        xSemaphoreGive(xUsbTxSemaphore);
    }

    xRobotStateMutex = xSemaphoreCreateMutexStatic(&s_state_mutex_struct);

    /* Record boot timestamp for deadman */
    s_last_jetson_packet_tick = 0;

    /* 4. Create Static Tasks with Rate Monotonic Priorities */
    xTaskCreateStatic(
        motor_task_entry,
        "MotorTask",
        MOTOR_TASK_STACK_SIZE,
        NULL,
        PRIORITY_MOTOR_TASK,
        s_motor_task_stack,
        &s_motor_task_tcb
    );

    xTaskCreateStatic(
        acquisition_task_entry,
        "AcqTask",
        ACQUISITION_TASK_STACK_SIZE,
        NULL,
        PRIORITY_ACQUISITION_TASK,
        s_acq_task_stack,
        &s_acq_task_tcb
    );

    xTaskCreateStatic(
        estimation_task_entry,
        "EstTask",
        ESTIMATION_TASK_STACK_SIZE,
        NULL,
        PRIORITY_ESTIMATION_TASK,
        s_est_task_stack,
        &s_est_task_tcb
    );

    xTaskCreateStatic(
        communication_task_entry,
        "CommTask",
        COMMUNICATION_TASK_STACK_SIZE,
        NULL,
        PRIORITY_COMMUNICATION_TASK,
        s_comm_task_stack,
        &s_comm_task_tcb
    );
}
