/**
 *    Computacao Embarcada - Computacao - Insper
 *
 *            Avaliacao Intermediaria
 *
 * Faça um firmware que permita a um usuário no computador acessar e configurar algumas
 * informações/ modos  de operação do microcontrolador. Essas funcionalidades devem ser
 * acessadas via comunicação serial (COM). Um menu deve informar ao usuário as possibilidades
 * e os comandos que devem ser digitados para operar o embarcado.
 *
 * Funcionalidades que o firmware deve possuir :
 *
 * 1. Exibir menu
 * 1. O usuário deve ser capaz de ligar/desligar o piscar led (led da placa)
 * 1. O usuário deve ser capaz de aumentar(+2 Hz) e diminuir (-2 Hz) a frequência do led
 *
 * Utilize o programa disponível no repositório (github.com/insper/Computacao-Embarcada/Avaliacoes/A1/)
 * como ponto de parida. O código deve fazer uso de interrupções e periféricos para gerenciar a
 * comunicação com o PC e o LED.
 *
 *  ## Extra (A)
 *
 *  1. O usuário deve ser capaz de ler o relógio do microcontrolador.
 *  1. O usuário deve ser capaz de entrar com um valor de frequência para o led de forma numérica no termina.
 *
 */

/************************************************************************/
/* Includes                                                              */
/************************************************************************/

#include "asf.h"
#include <string.h>

/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/

#define YEAR        2017
#define MOUNTH      3
#define DAY         27
#define WEEK        13
#define HOUR        9
#define MINUTE      5
#define SECOND      0

/**
 *  USART
 */
#define USART_COM     USART1
#define USART_COM_ID  ID_USART1

/************************************************************************/
/* LED                                                                     */
/************************************************************************/

#define LED_PIO_ID	 ID_PIOC
#define LED_PIO			 PIOC
#define LED_PIN			 8
#define LED_PIN_MASK (1<<LED_PIN)

/************************************************************************/
/* Variaveis globais                                                          */
/************************************************************************/

 /* buffer para recebimento de dados */
 uint8_t bufferRX[100];
 
 /* buffer para transmissão de dados */
 uint8_t bufferTX[100];
 
 /* Flag LED */
 int flag_led0 = 0;
 
 /* Frequencia LED */
 int freq = 1;
 
 /* Horario */
 uint32_t hora;
 uint32_t minuto;
 uint32_t segundo;
 
 


/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

void led_init(int estado);
uint32_t usart_puts(uint8_t *pstring);
uint32_t usart_gets(uint8_t *pstring);
void TC1_Handler(void);
void TC1_init(int frequencia);
void pin_toggle(Pio *pio, uint32_t mask);
void RTC_init(void);

/************************************************************************/
/* Handlers                                                             */
/************************************************************************/

void TC1_Handler(void){
	volatile uint32_t ul_dummy;

    /****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
    ******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
    if(flag_led0)
        pin_toggle(LED_PIO, LED_PIN_MASK);
}

/************************************************************************/
/* Funcoes                                                              */
/************************************************************************/

void led_init(int estado){
    pmc_enable_periph_clk(LED_PIO_ID);
    pio_set_output(LED_PIO, LED_PIN_MASK, 1, 0, 0 );
};

void pin_toggle(Pio *pio, uint32_t mask){
   if(pio_get_output_data_status(pio, mask))
    pio_clear(pio, mask);
   else
    pio_set(pio,mask);
}

void TC1_init(int frequencia){
    uint32_t ul_div;
    uint32_t ul_tcclks;
    uint32_t ul_sysclk = sysclk_get_cpu_hz();

    uint32_t channel = 1;

    /* Configura o PMC */
    pmc_enable_periph_clk(ID_TC1);

    /** Configura o TC para operar em  4Mhz e interrupçcão no RC compare */
    tc_find_mck_divisor(frequencia, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
    tc_init(TC0, channel, ul_tcclks | TC_CMR_CPCTRG);
    tc_write_rc(TC0, channel, (ul_sysclk / ul_div) / frequencia);

    /* Configura e ativa interrupçcão no TC canal 0 */
    NVIC_EnableIRQ((IRQn_Type) ID_TC1);
    tc_enable_interrupt(TC0, channel, TC_IER_CPCS);

    /* Inicializa o canal 0 do TC */
    tc_start(TC0, channel);
}

void RTC_init(){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(RTC, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(RTC, YEAR, MOUNTH, DAY, WEEK);
	rtc_set_time(RTC, HOUR, MINUTE, SECOND);
	
	/* Configure RTC interrupts */
    NVIC_DisableIRQ(RTC_IRQn);
    NVIC_ClearPendingIRQ(RTC_IRQn);
    NVIC_SetPriority(RTC_IRQn, 0);
    NVIC_EnableIRQ(RTC_IRQn);

    /* Ativa interrupcao via alarme */
    rtc_enable_interrupt(RTC,  RTC_IER_ALREN);
}

/**
 * \brief Configure USART peripheral
 */
static void USART1_init(void){

  /* Configura USART1 Pinos */
  sysclk_enable_peripheral_clock(ID_PIOB);
  sysclk_enable_peripheral_clock(ID_PIOA);
  pio_set_peripheral(PIOB, PIO_PERIPH_D, PIO_PB4);  // RX
  pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA21); // TX
  MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO4;

  /* Configura opcoes USART */
  const sam_usart_opt_t usart_settings = {
    .baudrate     = 115200,
    .char_length  = US_MR_CHRL_8_BIT,
    .parity_type  = US_MR_PAR_NO,
    .stop_bits    = US_MR_NBSTOP_1_BIT    ,
    .channel_mode = US_MR_CHMODE_NORMAL
  };

  /* Ativa Clock periferico USART0 */
  sysclk_enable_peripheral_clock(USART_COM_ID);

  /* Configura USART para operar em modo RS232 */
  usart_init_rs232(USART_COM, &usart_settings, sysclk_get_peripheral_hz());

  /* Enable the receiver and transmitter. */
	usart_enable_tx(USART_COM);
	usart_enable_rx(USART_COM);
}


/**
 *  Envia para o UART uma string
 */
uint32_t usart_puts(uint8_t *pstring){
  uint32_t i = 0 ;

  while(*(pstring + i)){
    usart_serial_putchar(USART_COM, *(pstring+i++));
    while(!uart_is_tx_empty(USART_COM)){};
  }
  return(i);
}

/**
 * Busca do UART uma mensagem enviada pelo computador terminada em \n
 */
uint32_t usart_gets(uint8_t *pstring){
  uint32_t i = 0 ;
  usart_serial_getchar(USART_COM, (pstring+i));
  while(*(pstring+i) != '\n'){
    usart_serial_getchar(USART_COM, (pstring+(++i)));
  }
  *(pstring+i+1)= 0x00;
  return(i);

}

/************************************************************************/
/* Main Code	                                                        */
/************************************************************************/
int main(void){
	/* Initialize the SAM system */
	sysclk_init();

	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	/* Inicializa led */
	led_init(0);

	/** Inicializa USART */
	USART1_init();
	
	/** Inicializa RTC */
	RTC_init();
  
    /** Configura timer 0 */
	TC1_init(freq);
	

  /** Super loop */
	while (1) {
		
		usart_puts("Menu:\n* Ligar pisca LED: l \n* Desligar pisca LED: d \n* Aumentar frequencia: + \n* Diminuir frequencia: - \n* Hora atual: h \n* Menu: m\n");
		delay_s(1);
		usart_puts(bufferTX);
		usart_gets(bufferRX);
	
		if (bufferRX[0]=='l'){
			usart_puts("Pisca LED ON\n");
			flag_led0 = 1;
			//pisca_led(1);
		}
		
		if (bufferRX[0]=='d'){
			usart_puts("Pisca LED OFF\n");
			flag_led0 = 0;
			pio_set_output(LED_PIO, LED_PIN_MASK, 1, 0, 0 );
			//pisca_led(0);
		}
		
		if (bufferRX[0]=='+'){
			freq += 2;
			TC1_init(freq);
			usart_puts("Aumentando frequencia em 2Hz\n");
		}
		
		if (bufferRX[0]=='-'){
			freq -= 2;
			TC1_init(freq);
			usart_puts("Diminuindo a frequencia em 2Hz\n");
		}
		
		if (bufferRX[0]=='h'){
			rtc_get_time(RTC, &hora, &minuto, &segundo);
			sprintf(bufferTX, "hora atual:\n %d:%d:%d \n", hora,minuto,segundo);
			usart_puts(bufferTX);
			
			
		}
		
		if (bufferRX[0]=='m'){
			usart_puts("Menu:\n* Ligar pisca LED: l \n* Desligar pisca LED: d \n* Aumentar frequencia: + \n* Diminuir frequencia: - \n* Hora atual: h \n* Menu: m\n");
		}	
	}
}
