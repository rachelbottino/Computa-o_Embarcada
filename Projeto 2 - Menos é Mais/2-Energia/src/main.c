#include "asf.h"

/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/

/**
 *  Informacoes para o RTC
 *  poderia ser extraida do __DATE__ e __TIME__
 *  ou ser atualizado pelo PC.
 */
#define YEAR        2017
#define MOUNTH      3
#define DAY         27
#define WEEK        13
#define HOUR        9
#define MINUTE      5
#define SECOND      0

/**
 * LEDs
 */
#define LED_PIO_ID		ID_PIOD
#define LED_PIO       PIOD
#define LED_PIN		    11
#define LED_PIN_MASK  (1<<LED_PIN)

/**
 * Botão
 */
#define BUT_PIO_ID      ID_PIOA
#define BUT_PIO         PIOA
#define BUT_PIN		    11
#define BUT_PIN_MASK    (1 << BUT_PIN)
#define BUT_DEBOUNCING_VALUE  79

/** 
 *  USART
 */
#define USART_COM     USART1
#define USART_COM_ID  ID_USART1

/************************************************************************/
/* VAR globais                                                          */
/************************************************************************/
volatile bool g_ledBlinkOn = false;
volatile uint32_t usart_transmission = 0;

 /* buffer para recebimento de dados */
 uint8_t bufferRX[100];
 
 /* buffer para transmissão de dados */
 uint8_t bufferTX[100];
 
 /* Frequencia LED */
 int freq = 2;
 

/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

void LED_init(int estado);
void TC1_init(int freq);
void RTC_init(void);
void pin_toggle(Pio *pio, uint32_t mask);
uint32_t usart_puts(uint8_t *pstring);
uint32_t usart_gets(uint8_t *pstring);

/************************************************************************/
/* Handlers                                                             */
/************************************************************************/

/**
 *  Handle Interrupcao USART
 */
void USART1_Handler(void){
	
	// If data arrived
	if((usart_get_status(USART_COM)) & US_IER_RXRDY) {
		
		usart_puts("Menu:\n* Ligar pisca LED: l \n* Desligar pisca LED: d \n");
		usart_gets(bufferRX);
		
		if (bufferRX[0]=='l'){
			usart_puts("Pisca LED ON\n");
			g_ledBlinkOn = true;
		}
		
		if (bufferRX[0]=='d'){
			usart_puts("Pisca LED OFF\n");
			g_ledBlinkOn = false;
			pio_set_output(LED_PIO, LED_PIN_MASK, 0, 0, 0 );
		}
		else {
			usart_transmission = 1;
		}
	}
}
/**
 *  Interrupt handler for TC1 interrupt. 
 */
void TC1_Handler(void){
	volatile uint32_t ul_dummy;

    /****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
    ******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);
		
	if(g_ledBlinkOn){
		pin_toggle(LED_PIO, LED_PIN_MASK);
	}
 }

/**
 * \brief Interrupt handler for the RTC. Refresh the display.
 */
void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	/* Second increment interrupt */
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
	
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);

	} else {
		/* Time or date alarm */
		if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
      
			rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
		}
	}
}


/************************************************************************/
/* Funcoes                                                              */
/************************************************************************/

/** 
 *  Toggle pin controlado pelo PIO (out)
 */
void pin_toggle(Pio *pio, uint32_t mask){
   if(pio_get_output_data_status(pio, mask)){
    pio_clear(pio, mask);
   }
   else {
    pio_set(pio,mask);
   }
}


/**
 * @Brief Inicializa o pino do LED
 */
void LED_init(int estado){
    pmc_enable_periph_clk(LED_PIO_ID);
    pio_set_output(LED_PIO, LED_PIN_MASK, estado, 0, 0 );
};

/**
 * Configura TimerCounter (TC0) para gerar uma interrupcao no canal 0-(ID_TC1) 
 * a cada 250 ms (4Hz)
 */
void TC1_init(int freq){   
    uint32_t ul_div;
    uint32_t ul_tcclks;
    uint32_t ul_sysclk = sysclk_get_cpu_hz();
    
    uint32_t channel = 1;
    
    /* Configura o PMC */
    pmc_enable_periph_clk(ID_TC1);    

    /** Configura o TC para operar em  4Mhz e interrupçcão no RC compare */
    tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
    tc_init(TC0, channel, ul_tcclks | TC_CMR_CPCTRG);
    tc_write_rc(TC0, channel, (ul_sysclk / ul_div) / freq);

    /* Configura e ativa interrupçcão no TC canal 0 */
    NVIC_EnableIRQ((IRQn_Type) ID_TC1);
    tc_enable_interrupt(TC0, channel, TC_IER_CPCS);

    /* Inicializa o canal 0 do TC */
    tc_start(TC0, channel);
}

/**
 * Configura o RTC para funcionar com interrupcao de alarme
 */
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
 * \brief Configure UART console.
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
  stdio_serial_init(CONF_UART, &usart_settings);

 }
 
 uint32_t usart_puts(uint8_t *pstring){
  uint32_t i = 0 ;

  while(*(pstring + i)){
    usart_serial_putchar(USART_COM, *(pstring+i++));
    while(!uart_is_tx_empty(USART_COM)){};
  }
  return(i);
}

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
	//sysclk_init();

	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;

	/* Configura Leds */
    LED_init(0);   
    
    /** Configura timer 0 */
    TC1_init(freq);
    
    /** Configura RTC */
    RTC_init();
  
	/** Inicializa USART como printf */
	USART1_init();
  
	while (1) {
		
		/* entra em modo sleep */
		pmc_enable_sleepmode(SLEEPMGR_SLEEP_WFI);
		pmc_enable_sleepmode(SUPC_CR_VROFF);
		
		if(usart_transmission) {
			usart_puts(bufferRX);
			usart_transmission = 0;
		}
	}
}
