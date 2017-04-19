/*
FreeRTOS V7.3.0 - Copyright (C) 2012 Real Time Engineers Ltd.

FEATURES AND PORTS ARE ADDED TO FREERTOS ALL THE TIME.  PLEASE VISIT 
http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

***************************************************************************
*                                                                       *
*    FreeRTOS tutorial books are available in pdf and paperback.        *
*    Complete, revised, and edited pdf reference manuals are also       *
*    available.                                                         *
*                                                                       *
*    Purchasing FreeRTOS documentation will not only help you, by       *
*    ensuring you get running as quickly as possible and with an        *
*    in-depth knowledge of how to use FreeRTOS, it will also help       *
*    the FreeRTOS project to continue with its mission of providing     *
*    professional grade, cross platform, de facto standard solutions    *
*    for microcontrollers - completely free of charge!                  *
*                                                                       *
*    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
*                                                                       *
*    Thank you for using FreeRTOS, and thank you for your support!      *
*                                                                       *
***************************************************************************


This file is part of the FreeRTOS distribution.

FreeRTOS is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License (version 2) as published by the
Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
>>>NOTE<<< The modification to the GPL is included to allow you to
distribute a combined work that includes FreeRTOS without being obliged to
provide the source code for proprietary components outside of the FreeRTOS
kernel.  FreeRTOS is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
more details. You should have received a copy of the GNU General Public
License and the FreeRTOS license exception along with FreeRTOS; if not it
can be viewed here: http://www.freertos.org/a00114.html and also obtained
by writing to Richard Barry, contact details for whom are available on the
FreeRTOS WEB site.

1 tab == 4 spaces!

***************************************************************************
*                                                                       *
*    Having a problem?  Start by reading the FAQ "My application does   *
*    not run, what could be wrong?"                                     *
*                                                                       *
*    http://www.FreeRTOS.org/FAQHelp.html                               *
*                                                                       *
***************************************************************************


http://www.FreeRTOS.org - Documentation, training, latest versions, license 
and contact details.  

http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
including FreeRTOS+Trace - an indispensable productivity tool.

Real Time Engineers ltd license FreeRTOS to High Integrity Systems, who sell 
the code with commercial support, indemnification, and middleware, under 
the OpenRTOS brand: http://www.OpenRTOS.com.  High Integrity Systems also
provide a safety engineered and independently SIL3 certified version under 
the SafeRTOS brand: http://www.SafeRTOS.com.
*/

#include "stm32f30x.h"

/* Pliki niezbêdne do obs³ugi freeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Do prostej obs³ugi peryferiów na p³ytce STM32F3DISCOVERY */
#include "stm32f3_discovery.h"

/* Do konfiguracji sprzêtowej */
#include "hw_config.h"

/* Do obs³ugi USB */
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
/*-----------------------------------------------------------*/

/* Funkcja konfiguruj¹ca sprzêt do obs³ugi freeRTOS */
static void prvSetupHardware( void );

// Struktura zawieraj¹ce uchwyty do kolejek
struct xQueueHandles{
  xQueueHandle LedHandle;
  xQueueHandle TempHandle;
  xQueueHandle StringHandle;
  xQueueHandle USBINHandle;
  xQueueHandle USBOUTHandle;
};
/*-----------------------------------------------------------*/

// Uchwyt do kolejki przychodz¹cych transmisji USB. Zadeklarowany tutaj ze wzglêdu na jego u¿ycie w usb_endp.c
xQueueHandle xUSBINQueue = NULL;

/* Prototypy watkow */
static portTASK_FUNCTION_PROTO( vUSBTask, pvParameters );
static portTASK_FUNCTION_PROTO( vLedTask, pvParameters );
static portTASK_FUNCTION_PROTO( vTempTask, pvParameters );
static portTASK_FUNCTION_PROTO( vStringTask, pvParameters );

/* Inicjalizacja USB, diod i przetwornika A/C */
void USB_Config( void );
void LED_Init( void );
void ADC_Config( void );
void SystemInit();

// Wymagane przez freeRTOS
unsigned long ulRunTimeStatsClock = 0UL;

uint8_t USB_Tx_Buffer[64];

GPIO_InitTypeDef GPIO_InitStruct;


int main( void )
{

  

  // Zmienne uchwytów do kolejek
  static xQueueHandle xLedQueue = NULL;
  static xQueueHandle xTempQueue = NULL;
  static xQueueHandle xUSBOUTQueue = NULL;
  static xQueueHandle xStringQueue = NULL;
  static struct xQueueHandles xHandles;
  
  /* Konfiguracja sprzetu */
  prvSetupHardware();
  LED_Init();
  USB_Config();
  ADC_Config();
  
  /* Tworzenie kolejek */
  xUSBINQueue = xQueueCreate( 255, ( unsigned portBASE_TYPE ) sizeof( uint8_t ) );
  xUSBOUTQueue = xQueueCreate( 255, ( unsigned portBASE_TYPE ) sizeof( uint8_t ) );
  xLedQueue = xQueueCreate( 10, ( unsigned portBASE_TYPE ) sizeof( uint8_t ) );
  xTempQueue = xQueueCreate( 10, ( unsigned portBASE_TYPE ) sizeof( uint8_t ) );
  xStringQueue = xQueueCreate( 255, ( unsigned portBASE_TYPE ) sizeof( uint8_t ) );
  
  /* Struktura zawierajaca adresy kolejek */
  xHandles.LedHandle = xLedQueue;
  xHandles.StringHandle = xStringQueue;
  xHandles.TempHandle = xTempQueue;
  xHandles.USBINHandle = xUSBINQueue;
  xHandles.USBOUTHandle = xUSBOUTQueue;
  
  /* 
  * Rejestr kolejek ulatwia debugowanie pogramu. 
  * Dzieki temu debugger wyposazony w obluge freeRTOS moze wyswietlic aktywne kolejki i ich zawartosc.
  * Jezeli configQUEUE_REGISTRY_SIZE jest mniejsze od 1 to rejestr nie powstanie.
  */
  vQueueAddToRegistry( xUSBINQueue, ( signed char * ) "USBINQueue" );
  vQueueAddToRegistry( xUSBOUTQueue, ( signed char * ) "USBOUTQueue" );
  vQueueAddToRegistry( xLedQueue, ( signed char * ) "LedQueue" );
  vQueueAddToRegistry( xTempQueue, ( signed char * ) "TempQueue" );
  vQueueAddToRegistry( xStringQueue, ( signed char * ) "StringQueue" );
  
  /* 
   * Tworzenie watkow 
   */
  xTaskCreate( vUSBTask, ( signed char * ) "usb", configMINIMAL_STACK_SIZE * 2, ( void * ) &xHandles, 3, NULL );
  xTaskCreate( vLedTask, ( signed char * ) "led", configMINIMAL_STACK_SIZE * 2, ( void * ) &xLedQueue, 3, NULL );
  xTaskCreate( vTempTask, ( signed char * ) "temp", configMINIMAL_STACK_SIZE * 2, ( void * ) &xHandles, 3, NULL );
  xTaskCreate( vStringTask, ( signed char * ) "string", configMINIMAL_STACK_SIZE * 2, ( void * ) &xHandles, 3, NULL );
  
  /* Uruchomienie w¹tków. */
  vTaskStartScheduler();
  
  /* 
   * Program nie powinien dotrzec do tego miejca. Moze sie to zdarzyc tylko w wypadku,
   * gdy brakuje RAMu dla utworzenia watku ja³owego
   */
  // Tutaj mo¿na dodaæ ewentualn¹ sygnalizacjê b³êdu
  for( ;; );
}
/*-----------------------------------------------------------*/

/**
* @brief  Konfiguracja USB
* @param  None
* @retval None
*/
void USB_Config(void)
{
  Set_System();
  Set_USBClock();
  USB_Interrupts_Config();
  
  USB_Init();
  
  while (bDeviceState != CONFIGURED)
  {}
}
/*-----------------------------------------------------------*/

/**
* @brief  Konfiguracja LED
* @param  None
* @retval None
*/
void LED_Init(void)
{
  STM_EVAL_LEDInit(LED3);
  STM_EVAL_LEDInit(LED4);
  STM_EVAL_LEDInit(LED5);
  STM_EVAL_LEDInit(LED6);
  STM_EVAL_LEDInit(LED7);
  STM_EVAL_LEDInit(LED8);
  STM_EVAL_LEDInit(LED9);
  STM_EVAL_LEDInit(LED10);
}
/*-----------------------------------------------------------*/

/**
* @brief  Konfiguracja ADC
* @param  None
* @retval None
*/
void ADC_Config(){
  // Struktury niezbêdne do inicjalizacji
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  
  // Konfiguracja zegara dla ADC1
  RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div2);
  // Uruchomienie zegara dla ADC1
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);
  
  // Inicjalizacja struktury
  ADC_StructInit(&ADC_InitStructure);

  // Uaktywnienie czujnika temperatury
  ADC_TempSensorCmd(ADC1, ENABLE);
  
  /* Kalibracja przetwornika */
  // Wybranie trybu kalibracji
  ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
  // Kalibracja
  ADC_StartCalibration(ADC1);
  // Oczekiwanie na zakoñczenie kalibracji
  while(ADC_GetCalibrationStatus(ADC1) != RESET );
     
  /* Opisy struktur znajduj¹ siê w pliku stm32f30x_adc.h */
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;                                                                    
  ADC_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode;                    
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;             
  ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_OneShot;                  
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;          
  ADC_CommonInit(ADC1, &ADC_CommonInitStructure);
  
  ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable;
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; 
  ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;         
  ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;   
  ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;  
  ADC_InitStructure.ADC_NbrOfRegChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);
  
  // Wybranie i konfiguracja kana³u ADC. W tym przypadku jest to kana³ 16, do którego pod³¹czony jest czujnik temp.
  ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_601Cycles5);
   
  // Uaktywnienie ADC1
  ADC_Cmd(ADC1, ENABLE);
  
  // Oczekiwanie na flagê RDY - gotowoœæ przetwornika do pracy
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));
}
/*-----------------------------------------------------------*/

/**
* @brief  Watek obslugi USB
* @param  Wskaznik void do struktury xQueueHandles 
* @retval None
*/
static portTASK_FUNCTION( vUSBTask, pvParameters )
{
  // Dane przychodz¹ce z kolejki xUSBINQueue - odebrane z portu USB
  uint8_t usData;
  // Dane przychodz¹ce z kolejki xUSBOUTQueue - do wys³ania do portu USB
  uint8_t outData;
  // D³ugoœæ stringa 
  size_t strLen = 0;
  // Do pêtli for
  int i = 0;
  
  /* Zamienienie wskaŸnika void * pvParameters na adres do struktury xQueueHandles */
  struct xQueueHandles *xHandlesPtr = pvParameters;
  struct xQueueHandles xHandles;
  xHandles = *xHandlesPtr;
  
  while(1){
    // Sprawdzenie czy w kolejce s¹ jakieœ wiadomoœci do odebrania
    if( uxQueueMessagesWaiting( xHandles.USBINHandle ) ){
      // Odebranie wiadomoœæi i zapisanie jej w zmiennej usData
      // oraz sprawdzenie czy wiadomoœæ zosta³a poprawnie odebrana
      if( xQueueReceive( xHandles.USBINHandle,  &usData, 0 ) == pdPASS ){
        /*
         * Sprawdzenie rodzaju polecenia wys³anego z komputera
         * 0x10, 0x20 i 0x30 - obs³uga LED
         * 0x40 - odebranie tekstu
         * 0x80 - pomiar temperatury
         */
        
        // W tym miejscu wykonywana jest operacja logiczna OR.
        if(usData|0x30){
          xQueueSend( xHandles.LedHandle, &usData, 0 );
        }
        // Maskowanie pierwszych czterech bitów i porównanie
        if((usData&0xF0) == 0x40){
          // Przekazanie kodu polecenia do w¹tku obs³ugi tekstu
          xQueueSend( xHandles.StringHandle, &usData, 0 );
          // Sprawdzenie czy po kodzie polecenia jest bajt d³ugoœci tekstu.
          if( uxQueueMessagesWaiting( xHandles.USBINHandle ) ){
            // Odebranie bajtu d³ugoœci tekstu i zapisanie do zmiennej strLen
            if( xQueueReceive( xHandles.USBINHandle, &strLen, 0 ) == pdPASS ){
              // Przes³anie bajtu d³ugoœci tekstu do w¹tku obs³ugi tekstu
              xQueueSend( xHandles.StringHandle, &strLen, 0 );
              // Odbieranie nastêpnych bajtów tworz¹cych string o d³ugoœci strLen
              for(i = strLen; i > 0; i--){
                if( xQueueReceive( xHandles.USBINHandle, &usData, 0 ) == pdPASS ){
                  // Je¿eli bajt zosta³ poprawnie odebrany zostanie przes³any do w¹tku obs³ugi tekstu
                  xQueueSend( xHandles.StringHandle, &usData, 0 );
                } else {
                  // W przypadku b³êdu nastêpuje wyjœcie z pêtli, poniewa¿ przewa¿nie oznacza to ¿e tekst jest krótszy
                  // ni¿ jest to przewidziane w zmiennej strLen
                  break;
                }
              }
            }
          }
        }
        // Maskowanie i porównanie
        if((usData&0xF0) == 0x80){
          // Przes³anie bajtu do w¹tku obs³ugi czujnika temperatury.
          // Jest to dla niego sygna³ do rozpoczêcia konwersji
          xQueueSend( xHandles.TempHandle, &usData, 0 );
        }
      }
    }
    
    // Sprawdzenie czy w kolejce xUSBOUTQueue jest wiadomoœæ
    if( uxQueueMessagesWaiting( xHandles.USBOUTHandle ) ){
      // Odebranie i zapisanie do zmiennej outData
      if( xQueueReceive( xHandles.USBOUTHandle,  &outData, 10 ) == pdPASS ){
        // Wys³anie do bufora USB.
        USB_Send_Data((char*)&outData, 1);
      }
    }		
  }
  
  /* 
  * Kazdy watek musi posiadac nieskonczona petle. 
  * Program nie powinien nigdy dotrzec do tej lini.
  */
}
/*-----------------------------------------------------------*/

/**
* @brief  Watek obslugi Led
* @param  Wskaznik void do kolejki Led 
* @retval None
*/
static portTASK_FUNCTION( vLedTask, pvParameters )
{
  /*
   * Ten w¹tek jest obs³ugiwany jednym bajtem danych.
   * Pierwsze cztery bity to rodzaj operacji.
   * Ostatnie trzy wybieraj¹ diodê.
   * 0x10 - Zapalenie diody
   * 0x20 - Zgaszenie diody
   * 0x30 - Prze³¹czenie ( odwrócenie stanu ) diody
   * 0x0 - Dioda LED3
   * 0x1 - Dioda LED4
   * ...
   * 0x7 - Dioda LED7
   */
  uint8_t usData;
  while(1){
    if( uxQueueMessagesWaiting( *( ( xQueueHandle * ) pvParameters ) ) ){
      if( xQueueReceive( *( ( xQueueHandle * ) pvParameters ),  &usData, 0 ) == pdPASS ){
        // Funkcja switch sprawdzaj¹ca rodzaj operacji
        switch(usData&0x30){
        case 0x10:
          STM_EVAL_LEDOn(usData&0x7);
          break;
        case 0x20:
          STM_EVAL_LEDOff(usData&0x7);
          break;
        case 0x30:
          STM_EVAL_LEDToggle(usData&0x7);
          break;
          default:
            break;
        }
      }
    }
  }
  /* 
  * Kazdy watek musi posiadac nieskonczona petle. 
  * Program nie powinien nigdy dotrzec do tej lini.
  */
}
/*-----------------------------------------------------------*/

/**
* @brief  Watek obslugi czujnika temperatury
* @param  Wskaznik void do struktury xQueueHandles 
* @retval None
*/
static portTASK_FUNCTION( vTempTask, pvParameters )
{
  uint8_t usData;
  struct xQueueHandles *xHandlesPtr = pvParameters;
  struct xQueueHandles xHandles;
  uint8_t templ = 0;
  uint8_t temph = 0;
  xHandles = *xHandlesPtr;
  
  while(1){
    if( uxQueueMessagesWaiting( xHandles.TempHandle ) ){
      if( xQueueReceive( xHandles.TempHandle, &usData, 0 ) == pdPASS ){
        if( usData == 0x80 ){
          // Rozpoczêcie konwersji
          ADC_StartConversion(ADC1);   
          // Oczekiwanie na zakoñczenie konwersji
          while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
          // Pobranie mniej znacz¹cego bajtu wartoœci pomiaru (wartoœæ ta ma 12 bitów)
          templ = ADC_GetConversionValue(ADC1)&0xFF;
          // Pobranie bardziej znacz¹cego bajtu wartoœci pomiaru
          temph = ADC_GetConversionValue(ADC1)&0xFF00;
          // Przesz³anie kodu operacji. W tym wypadku 0x80 oznaczaj¹ce pomiar temperatury.
          xQueueSend(xHandles.USBOUTHandle, &usData, 0);
          // Przes³anie mniej znacz¹cego bajtu wartoœci pomiaru
          xQueueSend(xHandles.USBOUTHandle, &templ, 0);
          // Przes³anie bardziej znacz¹cego bajtu wartoœci pomiaru.
          xQueueSend(xHandles.USBOUTHandle, &temph, 0);
          // Wyczyszczenie flagi EOC (End Of Conversion - Koniec Konwersji)
          ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
        }
      }
    } 
  }
  /* 
  * Kazdy watek musi posiadac nieskonczona petle. 
  * Program nie powinien nigdy dotrzec do tej lini.
  */
}
/*-----------------------------------------------------------*/

/**
* @brief  Watek odbierania i wysylania stringow
* @param  Wskaznik void do struktury xQueueHandles 
* @retval None
*/
static portTASK_FUNCTION( vStringTask, pvParameters )
{
  /*
   * Ten w¹tek jedynie przekierowuje bajty przychodz¹ce do kolejki xStringQueue do kolejki xUSBOUTQueue.
   * Mo¿na dodaæ zapisywanie do bufora i ewentualne przetwarzanie tekstu.
   */
  uint8_t usData;
  struct xQueueHandles *xHandlesPtr = pvParameters;
  struct xQueueHandles xHandles;
  xHandles = *xHandlesPtr;
  
  while(1){	
    if( uxQueueMessagesWaiting( xHandles.StringHandle ) ){
      if( xQueueReceive( xHandles.StringHandle, &usData, 0 ) == pdPASS ){
        // Przes³anie do xUSBOURQueue czyli wys³anie spowrotem na port USB.
        xQueueSend( xHandles.USBOUTHandle, &usData, 0 );
      }
    } 
  }
  
  /* 
  * Kazdy watek musi posiadac nieskonczona petle. 
  * Program nie powinien nigdy dotrzec do tej lini.
  */
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
  /*
   * To jest standardowa funkcja freeRTOS. 
   */
  
  /* Start with the clocks in their expected state. */
  RCC_DeInit();
  
  /* Enable HSE (high speed external clock). */
  RCC_HSEConfig( RCC_HSE_ON );
  
  /* Wait till HSE is ready. */
  while( RCC_GetFlagStatus( RCC_FLAG_HSERDY ) == RESET )
  {
  }
  
  /* 2 wait states required on the flash. */
  *( ( unsigned long * ) 0x40022000 ) = 0x02;
  
  /* HCLK = SYSCLK */
  RCC_HCLKConfig( RCC_SYSCLK_Div1 );
  
  /* PCLK2 = HCLK */
  RCC_PCLK2Config( RCC_HCLK_Div1 );
  
  /* PCLK1 = HCLK/2 */
  RCC_PCLK1Config( RCC_HCLK_Div2 );
  
  /* PLLCLK = (8MHz / 2 ) * 5 = 20.0 MHz. */
  RCC_PLLConfig( RCC_PLLSource_HSI_Div2, RCC_PLLMul_5 );
  
  /* Enable PLL. */
  RCC_PLLCmd( ENABLE );
  
  /* Wait till PLL is ready. */
  while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
  {
  }
  
  /* Select PLL as system clock source. */
  RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK );
  
  /* Wait till PLL is used as system clock source. */
  while( RCC_GetSYSCLKSource() != 0x08 )
  {
  }
  
  /* Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE and AFIO clocks */
  RCC_APB2PeriphClockCmd(	RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB |RCC_AHBPeriph_GPIOC
                         | RCC_AHBPeriph_GPIOD | RCC_AHBPeriph_GPIOE | RCC_AHBPeriph_GPIOF, ENABLE );
  
  /* Set the Vector Table base address at 0x08000000 */
  NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x0 );
  
  NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
  
  /* Configure HCLK clock as SysTick clock source. */
  SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
  /* Ta funkcja jest wywo³ywana w przypadku przepe³enienia stosu w¹tków. */
  
  ( void ) pxTask;
  ( void ) pcTaskName;
  
  for( ;; );
}
/*-----------------------------------------------------------*/
void vApplicationTickHook(void)
{
  /* Wywo³ywane co SysTick */
}

void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {}
}
