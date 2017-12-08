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
#define LED_PIO_ID		ID_PIOC
#define LED_PIO			PIOC
#define LED_PIN		    8
#define LED_PIN_MASK	(1<<LED_PIN)

#define LED_PIO_ID		ID_PIOC
#define LED_PIO         PIOC
#define LED_PIN		    8
#define LED_PIN_MASK    (1<<LED_PIN)

#define LED1_PIO_ID    ID_PIOA
#define LED1_PIO       PIOA
#define LED1_PIN       0
#define LED1_PIN_MASK  (1 << LED1_PIN)

#define LED2_PIO_ID    ID_PIOC
#define LED2_PIO       PIOC
#define LED2_PIN       30
#define LED2_PIN_MASK  (1 << LED2_PIN)

#define LED3_PIO_ID    ID_PIOB
#define LED3_PIO       PIOB
#define LED3_PIN       2
#define LED3_PIN_MASK  (1 << LED3_PIN)

/**
 * Botão
 */
#define BUT_PIO_ID      ID_PIOA
#define BUT_PIO         PIOA
#define BUT_PIN		    11
#define BUT_PIN_MASK    (1 << BUT_PIN)
#define BUT_DEBOUNCING_VALUE  79

#define BUT1_PIO_ID		ID_PIOD
#define BUT1_PIO		PIOD
#define BUT1_PIN		28
#define BUT1_PIN_MASK	(1 << BUT1_PIN)
#define BUT1_DEBOUNCING_VALUE  79

#define BUT2_PIO_ID		ID_PIOC
#define BUT2_PIO		PIOC
#define BUT2_PIN		31
#define BUT2_PIN_MASK	(1 << BUT2_PIN)
#define BUT2_DEBOUNCING_VALUE  79

#define BUT3_PIO_ID		ID_PIOA
#define BUT3_PIO		PIOA
#define BUT3_PIN		19
#define BUT3_PIN_MASK	(1 << BUT3_PIN)
#define BUT3_DEBOUNCING_VALUE  79

/************************************************************************/
/* VAR globais                                                          */
/************************************************************************/
volatile uint8_t flag_led0 = 1;
/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

void BUT_init(Pio *p_pio,const uint32_t ul_mask, const uint32_t ul_attribute,
				uint32_t ul_id, uint32_t ul_attr,void (*p_handler) (uint32_t, uint32_t));void LED_init(Pio *pio, uint32_t pio_id, uint32_t mask, int estado);
void TC_init(Tc *TC, uint32_t ID_TC,  uint32_t channel, uint32_t freq);
void RTC_init(void);
void pin_toggle(Pio *pio, uint32_t mask);

/************************************************************************/
/* Handlers                                                             */
/************************************************************************/

/**
 *  Handle Interrupcao botao 1
 */
void but1_Handler(){
	/*
	*  limpa interrupcao do PIO
	*/
	uint32_t pioIntStatus;
	pioIntStatus =  pio_get_interrupt_status(BUT1_PIO);
    
	/**
	*  Toggle status led
	*/
	if(pio_get_output_data_status(LED1_PIO, LED1_PIN_MASK)){
		pio_clear(LED1_PIO, LED1_PIN_MASK);
		tc_start(TC0, 1);
	}
	else{
		pio_set(LED1_PIO,LED1_PIN_MASK);
		tc_stop(TC0, 1);
	}
}
/**
 *  Handle Interrupcao botao 2
 */
void but2_Handler(){
	/*
	*  limpa interrupcao do PIO
	*/
	uint32_t pioIntStatus;
	pioIntStatus =  pio_get_interrupt_status(BUT2_PIO);
    
	/**
	*  Toggle status led
	*/
	if(pio_get_output_data_status(LED2_PIO, LED2_PIN_MASK)){
		pio_clear(LED2_PIO, LED2_PIN_MASK);
		tc_start(TC0, 2);
	}
   
	else{
		pio_set(LED2_PIO,LED2_PIN_MASK);
		tc_stop(TC0, 2);
	}
}
/**
 *  Handle Interrupcao botao 1
 */
void but3_Handler(){
	/*
	*  limpa interrupcao do PIO
	*/
	uint32_t pioIntStatus;
	pioIntStatus =  pio_get_interrupt_status(BUT3_PIO);
    
	/**
	*  Toggle status led
	*/
	if(pio_get_output_data_status(LED3_PIO, LED3_PIN_MASK)){
		pio_clear(LED3_PIO, LED3_PIN_MASK);
		tc_start(TC1, 0);
	}
   
	else{
		pio_set(LED3_PIO,LED3_PIN_MASK);
		tc_stop(TC1, 0);
	}
}
/**
 *  Interrupt handler for TC0 interrupt. 
 */
void TC0_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	pin_toggle(LED_PIO, LED_PIN_MASK);
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

	/** Muda o estado do LED */
	pin_toggle(LED1_PIO, LED1_PIN_MASK);
}
/**
 *  Interrupt handler for TC2 interrupt. 
 */
void TC2_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 2);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	pin_toggle(LED2_PIO, LED2_PIN_MASK);
}
/**
 *  Interrupt handler for TC4 interrupt. 
 */
void TC3_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC1, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	pin_toggle(LED3_PIO, LED3_PIN_MASK);
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
	}
	else{
		/* Time or date alarm */
		if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {   
      
		/*Atualiza hora */       
		uint32_t ano,mes,dia,hora,minuto,segundo;
            
		rtc_get_date(RTC, &ano, &mes, &dia, NULL);
		rtc_get_time(RTC, &hora, &minuto, &segundo);
      
		/* incrementa minuto */
		if(minuto>=59){
			minuto = 0;
			if(hora>=23){
				hora=0;
				if(dia>=30){
					dia =1;
					if(mes>=12){
						mes =1;
						ano++;
					}
					else{
						mes++;
					}	 
				}
				else{
					dia++;
				}
			}
			else{
				hora++;
			}
		}
		else{
			minuto++;
		}
	  
		/* configura novo alarme do RTC */
		rtc_set_date_alarm(RTC, 1, mes, 1, dia);
		rtc_set_time_alarm(RTC, 1, hora, 1, minuto, 1, segundo);
     
		/* inverte status led */
		flag_led0 ^= 1;
     
		/* Ativa/desativa o TimerCounter */
		if(flag_led0){
			tc_start(TC0, 0);
			tc_start(TC0, 1);
			tc_start(TC0, 2);
			tc_start(TC1, 0);
		}
		else{  
			tc_stop(TC0, 0);
			tc_stop(TC0, 1);
			tc_stop(TC0, 2);
			tc_stop(TC1, 0);
		}
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
	if(pio_get_output_data_status(pio, mask))
		pio_clear(pio, mask);
	else
		pio_set(pio,mask);
}

/**
 * @Brief Inicializa o pino do BUT
 */
void BUT_init(Pio *p_pio,const uint32_t ul_mask, const uint32_t ul_attribute, 
				uint32_t ul_id, uint32_t ul_attr,void (*p_handler) (uint32_t, uint32_t)){
	/* config. pino botao em modo de entrada */
	pmc_enable_periph_clk(ul_id);
	pio_set_input(p_pio, ul_mask, ul_attribute);
    
	/* config. interrupcao em borda de descida no botao do kit */
	/* indica funcao (but_Handler) a ser chamada quando houver uma interrupção */
	pio_enable_interrupt(p_pio, ul_mask);
	pio_handler_set(p_pio, ul_id, ul_mask, ul_attr, p_handler);
    
	/* habilita interrupçcão do PIO que controla o botao */
	/* e configura sua prioridade                        */
	NVIC_EnableIRQ(ul_id);
	NVIC_SetPriority(ul_id, 1);
};

/**
 * @Brief Inicializa o pino do LED
 */
void LED_init(Pio *pio, uint32_t pio_id, uint32_t mask, int estado){
	pmc_enable_periph_clk(pio_id);
	pio_set_output(pio, mask, estado, 0, 0 );
};

/**
 * Configura TimerCounter para gerar uma interrupcao em um canal, a uma certa frequencia
 */
void TC_init( Tc *TC, uint32_t ID_TC,  uint32_t channel, uint32_t freq ){   
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();
     
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_TC);    

	/** Configura o TC para operar em  4Mhz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, channel, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, channel, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrupçcão no TC canal 0 */
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, channel, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, channel);
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

/************************************************************************/
/* Main Code	                                                        */
/************************************************************************/
int main(void){
	/* Initialize the SAM system */
	sysclk_init();

	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;

	/* Configura Leds */
	LED_init(LED_PIO, LED_PIO_ID,LED_PIN_MASK,0);
	LED_init(LED1_PIO, LED1_PIO_ID, LED1_PIN_MASK,0);
	LED_init(LED2_PIO, LED2_PIO_ID, LED2_PIN_MASK,0);
	LED_init(LED3_PIO, LED3_PIO_ID, LED3_PIN_MASK,0);
		
	/* Configura os botões */
	BUT_init(BUT1_PIO,BUT1_PIN_MASK,PIO_PULLUP | PIO_DEBOUNCE,BUT1_PIO_ID,PIO_IT_FALL_EDGE, but1_Handler);
	BUT_init(BUT2_PIO,BUT2_PIN_MASK,PIO_PULLUP | PIO_DEBOUNCE,BUT2_PIO_ID,PIO_IT_FALL_EDGE, but2_Handler);
	BUT_init(BUT3_PIO,BUT3_PIN_MASK,PIO_PULLUP | PIO_DEBOUNCE,BUT3_PIO_ID,PIO_IT_FALL_EDGE, but3_Handler);    
    
	/** Configura os TCs */
	TC_init(TC0, ID_TC0, 0, 4);
	TC_init(TC0, ID_TC1, 1, 8);
	TC_init(TC0, ID_TC2, 2, 11);
	TC_init(TC1, ID_TC3, 0, 17);
  
	/** Configura RTC */
	RTC_init();

	/* configura alarme do RTC */    
	rtc_set_date_alarm(RTC, 1, MOUNTH, 1, DAY);
	rtc_set_time_alarm(RTC, 1, HOUR, 1, MINUTE+1, 1, SECOND);
          
	while (1) {
		/* Entra em modo sleep */
		
	}
}