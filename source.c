/***************************************************************************//**
* @file main.c
* @brief main() function.
*****************************************************************************
* # License
* <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
*****************************************************************************
*
* The licensor of this software is Silicon Laboratories Inc. Your use of this
* software is governed by the terms of Silicon Labs Master Software License
* Agreement (MSLA) available at
* www.silabs.com/about-us/legal/master-software-license-agreement. This
* software is distributed to you in Source Code format and is governed by the
* sections of the MSLA applicable to Source Code.
*
******************************************************************************/
#include "sl_component_catalog.h"
#include "sl_system_init.h"
#include "app.h"
#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
#include "sl_power_manager.h"
#endif
#if defined(SL_CATALOG_KERNEL_PRESENT)
#include "sl_system_kernel.h"
#else // SL_CATALOG_KERNEL_PRESENT
#include "sl_system_process_action.h"
#endif // SL_CATALOG_KERNEL_PRESENT

#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"

#define BSP_TXPORT gpioPortA
#define BSP_RXPORT gpioPortA
#define BSP_TXPIN 5
#define BSP_RXPIN 6
#define BSP_ENABLE_PORT gpioPortD
#define BSP_ENABLE_PIN 4


#define BSP_GPIO_LEDS
#define BSP_GPIO_LED0_PORT gpioPortD
#define BSP_GPIO_LED0_PIN 2
#define BSP_GPIO_LED1_PORT gpioPortD
#define BSP_GPIO_LED1_PIN 3
#define BSP_GPIO_PB0_PORT gpioPortB
#define BSP_GPIO_PB0_PIN 0
#define BSP_GPIO_PB1_PORT gpioPortB
#define BSP_GPIO_PB1_PIN 1


#define LED0_ON 0x01
#define LED0_OFF 0x02
#define LED1_ON 0x03
#define LED1_OFF 0x04
#define BOTH_LEDS_ON 0x05
#define BOTH_LEDS_OFF 0x06


/**************************************************************************//**
* @brief
*    GPIO initialization
*****************************************************************************/
void initGPIO(void)
{
 // Configure the USART TX pin to the board controller as an output
 GPIO_PinModeSet(BSP_TXPORT, BSP_TXPIN, gpioModePushPull, 1);

 // Configure the USART RX pin to the board controller as an input
 GPIO_PinModeSet(BSP_RXPORT, BSP_RXPIN, gpioModeInput, 0);

 /*
  * Configure the BCC_ENABLE pin as output and set high.  This enables
  * the virtual COM port (VCOM) connection to the board controller and
  * permits serial port traffic over the debug connection to the host
  * PC.
  *
  * To disable the VCOM connection and use the pins on the kit
  * expansion (EXP) header, comment out the following line.
  */
 GPIO_PinModeSet(BSP_ENABLE_PORT, BSP_ENABLE_PIN, gpioModePushPull, 1);

 /*
  *
  * Hàm khởi tạo mode và interrupt của Button & LED
  *
  */
 // Enable GPIO clock
 CMU_ClockEnable(cmuClock_GPIO, true);
 // Configure PB0 and PB1 as input with glitch filter enabled
 GPIO_PinModeSet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN,
 gpioModeInputPullFilter, 1);
 GPIO_PinModeSet(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN,
 gpioModeInputPullFilter, 1);
 // Configure LED0 and LED1 as output
 GPIO_PinModeSet(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN,
 gpioModePushPull, 0);
 GPIO_PinModeSet(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN,
 gpioModePushPull, 0);
 // Enable IRQ for even numbered GPIO pins
 NVIC_EnableIRQ(GPIO_EVEN_IRQn);
 // Enable IRQ for odd numbered GPIO pins
 NVIC_EnableIRQ(GPIO_ODD_IRQn);
 // Enable falling-edge interrupts for PB pins
 GPIO_ExtIntConfig(BSP_GPIO_PB0_PORT,
 BSP_GPIO_PB0_PIN,BSP_GPIO_PB0_PIN, 0, 1, true);
 GPIO_ExtIntConfig(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN,
 BSP_GPIO_PB1_PIN, 0, 1, true);


}

/**************************************************************************//**
* @brief
*    USART0 initialization
*****************************************************************************/
void initUSART0(void)
{
 // Default asynchronous initializer (115.2 Kbps, 8N1, no flow control)
 //USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;
 USART_InitAsync_TypeDef init;

 init.enable = usartEnable;
 init.refFreq = 0;
 init.baudrate = 115200;
 init.oversampling = usartOVS16;
 init.databits = usartDatabits8;
 init.parity = USART_FRAME_PARITY_NONE;
 init.stopbits = usartStopbits1;

 init.mvdis = false;
 init.prsRxEnable = false;
 init.prsRxCh = 0;

 init.autoCsEnable = false;
 init.csInv = false;
 init.autoCsHold = 0;
 init.autoCsSetup = 0;
 init.hwFlowControl = usartHwFlowControlNone;

 // Route USART0 TX and RX to the board controller TX and RX pins
 GPIO->USARTROUTE[0].TXROUTE = (BSP_TXPORT << _GPIO_USART_TXROUTE_PORT_SHIFT)
           | (BSP_TXPIN << _GPIO_USART_TXROUTE_PIN_SHIFT);
 GPIO->USARTROUTE[0].RXROUTE = (BSP_RXPORT << _GPIO_USART_RXROUTE_PORT_SHIFT)
           | (BSP_RXPIN << _GPIO_USART_RXROUTE_PIN_SHIFT);

 // Enable RX and TX signals now that they have been routed
 GPIO->USARTROUTE[0].ROUTEEN = GPIO_USART_ROUTEEN_RXPEN | GPIO_USART_ROUTEEN_TXPEN;

 // Configure and enable USART0
 USART_InitAsync(USART0, &init);
}

int main(void)
{
 // Initialize Silicon Labs device, system, service(s) and protocol stack(s).
 // Note that if the kernel is present, processing task(s) will be created by
 // this call.
 sl_system_init();

 // Initialize the application. For example, create periodic timer(s) or
 // task(s) if the kernel is present.
 app_init();

#if defined(SL_CATALOG_KERNEL_PRESENT)
 // Start the kernel. Task(s) created in app_init() will start running.
 sl_system_kernel_start();
#else // SL_CATALOG_KERNEL_PRESENT

 uint8_t buffer;

 // Initialize GPIO and USART0
 initGPIO();
 initUSART0();

 while (1) {
   // Do not remove this call: Silicon Labs components process action routine
   // must be called from the super loop.
   sl_system_process_action();

   // Application process.
   app_process_action();

 // Zero out buffer
   buffer = 0;
  // Receive BUFLEN characters unless a new line is received first
  do
  {
    // Wait for a character
    buffer = USART_Rx(USART0);
  }
  while ( buffer == 0 );

   switch (buffer) {
     case LED0_ON:
       GPIO_PinOutSet(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN);
       break;
     case LED0_OFF:
       GPIO_PinOutClear(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN);
       break;
     case LED1_ON:
       GPIO_PinOutSet(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN);
       break;
     case LED1_OFF:
       GPIO_PinOutClear(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN);
       break;
     case BOTH_LEDS_ON:
       GPIO_PinOutSet(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN);
       GPIO_PinOutSet(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN);
       break;
     case BOTH_LEDS_OFF:
       GPIO_PinOutClear(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN);
       GPIO_PinOutClear(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN);
       break;
     default:
   }

  // Output received characters
  //USART_Tx(USART0, buffer);

#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
   // Let the CPU go to sleep if the system allows it.
   sl_power_manager_sleep();
#endif
 }
#endif // SL_CATALOG_KERNEL_PRESENT
}
