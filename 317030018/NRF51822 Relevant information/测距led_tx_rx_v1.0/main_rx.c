//深圳市弘毅云佳科技有限公司
//技术支持0755-86165842
#include <stdint.h>
#include <stdbool.h>
#include "radio_config.h"
#include "nrf_gpio.h"
#include "boards.h"

static uint8_t volatile packet[4];  ///< Received packet buffer
static uint32_t count = 0;

static uint8_t packet_tx[4];  ///< Packet to transmit
static uint8_t shuju = 0;
void init(void)
{
  /* Start 16 MHz crystal oscillator */
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;

  /* Wait for the external oscillator to start up */
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) 
  {
  }

  nrf_gpio_range_cfg_output(LED_START, LED_STOP);

  // Set radio configuration parameters
  radio_configure();
}

/**
 * main() function
 * @return 0. int return type required by ANSI/ISO standard.
 */
int main(void)
{
  init();

  while(true)
  {
		
		/************************************发送部分********************************************/		
		// Set tx payload pointer
    NRF_RADIO->PACKETPTR = (uint32_t)packet_tx;
				packet_tx[0] = shuju;
    NRF_RADIO->EVENTS_READY = 0U;
    NRF_RADIO->TASKS_TXEN = 1U;
    while (NRF_RADIO->EVENTS_READY == 0U)
    {
    }
    NRF_RADIO->TASKS_START = 1U;
    NRF_RADIO->EVENTS_END = 0U;  
    while(NRF_RADIO->EVENTS_END == 0U)
    {
    }
    NRF_RADIO->EVENTS_DISABLED = 0U;
    // Disable radio
    NRF_RADIO->TASKS_DISABLE = 1U;
    while(NRF_RADIO->EVENTS_DISABLED == 0U)
    {
    }
//		shuju = shuju^0x01;
//		nrf_delay_ms(300);
/************************************接收部分***********************************/
    // Set payload pointer
    NRF_RADIO->PACKETPTR = (uint32_t)packet;
		
    NRF_RADIO->EVENTS_READY = 0U;
    // Enable radio and wait for ready
    NRF_RADIO->TASKS_RXEN = 1U;
    while(NRF_RADIO->EVENTS_READY == 0U)
    {
    }
    NRF_RADIO->EVENTS_END = 0U;
    // Start listening and wait for address received event
    NRF_RADIO->TASKS_START = 1U;
    // Wait for end of packet
    while(NRF_RADIO->EVENTS_END == 0U)
    {
			count++;
			if(count >= 165500)
			{
				//count = 0;
				//没有收到数据
				packet[0] = 0xff;
				count = 0;
				nrf_gpio_pin_clear(LED0);
        nrf_gpio_pin_clear(LED1);
				break;
			}
    }
    // Write received data to LED0 and LED1 on CRC match
    if (NRF_RADIO->CRCSTATUS == 1U)
    {
      if( (packet[0] == 0) || (packet[0] == 1) )
      {
          nrf_gpio_pin_set(LED0);
					nrf_gpio_pin_set(LED1);
//					nrf_gpio_pin_clear(LED0);
//          nrf_gpio_pin_clear(LED1);

//         case 1:
//           nrf_gpio_pin_set(LED1);
//           nrf_gpio_pin_clear(LED0);
//           break;
      }
//			shuju = shuju^0x01;
    }
    NRF_RADIO->EVENTS_DISABLED = 0U;
    // Disable radio
    NRF_RADIO->TASKS_DISABLE = 1U;
    while(NRF_RADIO->EVENTS_DISABLED == 0U)
    {
    }
  }
}

/**
 *@}
 **/

