#include <stdint.h>
#include <stdbool.h>

/* Required hardware headers for interrupt IDs/constants */
#include "inc/hw_memmap.h"
#include "inc/hw_can.h"    /* provides CAN_INT_INTID_STATUS etc. */
#include "inc/hw_ints.h"   /* provides INT_CAN0 */
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/can.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"

uint32_t ui32Status;

#define COUNTER_CARD_CAN_ID        0x0200
#define PULSE_WIDTH_MS   120
static uint8_t sRXBufCounter[8];
static tCANMsgObject sRXMsgObjCounter;
volatile uint8_t pulse_active_mask = 0;

/* Decoded output shadow (what CPU commanded) */
static volatile uint16_t counter_bits_shadow = 0;
static volatile bool counter_update_flag = false;

//void Update_GPIO_From_CAN(uint8_t byte0)
//{
//    // Bit masks
//    uint8_t sos              = (byte0 >> 0) & 0x01;
//    uint8_t brake            = (byte0 >> 1) & 0x01;
//    uint8_t override_select  = (byte0 >> 2) & 0x01;
//    uint8_t biu_isolation    = (byte0 >> 3) & 0x01;
//    uint8_t trip_mode        = (byte0 >> 4) & 0x01;
//
//    // Write GPIO pins
//    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, sos ? GPIO_PIN_0 : 0);
//    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, brake ? GPIO_PIN_1 : 0);
//    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, override_select ? GPIO_PIN_2 : 0);
//    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, biu_isolation ? GPIO_PIN_3 : 0);
//    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, trip_mode ? GPIO_PIN_4 : 0);
//}
void Update_GPIO_From_CAN(uint8_t byte0)
{
    uint8_t mask = 0;

    if (byte0 & (1 << 0)) mask |= GPIO_PIN_0;   // SOS
    if (byte0 & (1 << 1)) mask |= GPIO_PIN_1;   // Brake
    if (byte0 & (1 << 2)) mask |= GPIO_PIN_2;   // Override
    if (byte0 & (1 << 3)) mask |= GPIO_PIN_3;   // BIU
    if (byte0 & (1 << 4)) mask |= GPIO_PIN_4;   // Trip

    if(mask)
    {
        pulse_active_mask = mask;

        /* Set pins HIGH */
        GPIOPinWrite(GPIO_PORTN_BASE,
                     GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 |
                     GPIO_PIN_3 | GPIO_PIN_4,
                     mask);

        /* Restart one-shot timer */
        TimerDisable(TIMER0_BASE, TIMER_A);
        TimerEnable(TIMER0_BASE, TIMER_A);
    }
}

void GPIO_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION));

    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE,
                          GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 |
                          GPIO_PIN_3 | GPIO_PIN_4);
}
void Timer0_Init(uint32_t sysclk)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));

    TimerDisable(TIMER0_BASE, TIMER_A);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_ONE_SHOT);

    uint32_t load = (sysclk / 1000) * PULSE_WIDTH_MS;
    TimerLoadSet(TIMER0_BASE, TIMER_A, load - 1);

    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

/* Prototype for the ISR (name must match vector: CANIntHandler) */
void CANIntHandler(void);

int main(void)
{
    uint32_t ui32SysClock;

    /* Configure system clock: PLL -> 120 MHz (adjust if needed) */
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_OSC_MAIN |
                                       SYSCTL_XTAL_25MHZ |
                                       SYSCTL_USE_PLL |
                                       SYSCTL_CFG_VCO_480),
                                      120000000);

    /* Enable GPIOA and CAN0 peripheral */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN0)) { }

    /* Configure PA0 = CAN0RX, PA1 = CAN0TX */
    GPIOPinConfigure(GPIO_PA0_CAN0RX);
    GPIOPinConfigure(GPIO_PA1_CAN0TX);
    GPIOPinTypeCAN(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    GPIO_Init();
    Timer0_Init(ui32SysClock);

    /* Initialize CAN controller and set bit rate (500 kbps) */
    CANInit(CAN0_BASE);
    CANBitRateSet(CAN0_BASE, ui32SysClock, 500000);

    /* Prepare message object for OUTPUT CARD RX */
    sRXMsgObjCounter.ui32MsgID       = COUNTER_CARD_CAN_ID;
    sRXMsgObjCounter.ui32MsgIDMask   = 0;   /* exact match */
    sRXMsgObjCounter.ui32Flags       = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_EXTENDED_ID;
    sRXMsgObjCounter.ui32MsgLen      = 8;
    sRXMsgObjCounter.pui8MsgData     = sRXBufCounter;

    /* Assign RX message object (use a free slot, e.g. 12) */
    CANMessageSet(CAN0_BASE, 16, &sRXMsgObjCounter, MSG_OBJ_TYPE_RX);

    /* Enable CAN interrupts (master + error/status) and NVIC entry */
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
    IntEnable(INT_CAN0);

    /* Enable CAN module */
    CANEnable(CAN0_BASE);

    /* Idle loop; ISR handles reception */
    while (1)
    {}
}

/* CAN0 interrupt handler */
void CANIntHandler(void)
{
//    uint32_t ui32Status;

    /* Get cause of interrupt */
    ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

    /* Controller status interrupt (error) */
    if (ui32Status == CAN_INT_INTID_STATUS)
    {
        /* Read controller status to clear interrupt; ignore details here */
        (void)CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);
    }
    /* Message object 2 caused the interrupt (RX complete) */
    else if (ui32Status == 16)   /* COUNTER CARD RX */
    {
        CANMessageGet(CAN0_BASE, 16, &sRXMsgObjCounter, true);

        /* Validate message type */
            /* Byte 1 = Outputs 1–8, Byte 2 = Outputs 9–16 */
            counter_bits_shadow =
                (uint16_t)sRXBufCounter[1] |
                ((uint16_t)sRXBufCounter[2] << 8);

            counter_update_flag = true;

            uint8_t byte0 = sRXBufCounter[0];   // First byte of payload
            Update_GPIO_From_CAN(byte0);

        CANIntClear(CAN0_BASE, 16);
    }
    else
    {
        /* Spurious / unexpected interrupt */
    }
}
void Timer0AIntHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    /* Clear only active pulse pins */
    GPIOPinWrite(GPIO_PORTN_BASE,
                 GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 |
                 GPIO_PIN_3 | GPIO_PIN_4,
                 0);

    pulse_active_mask = 0;
}

