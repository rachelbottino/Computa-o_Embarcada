/************************************************************************
* Rafael Corsi   - Insper
* rafael.corsi@insper.edu.br
*
* Computação Embarcada
*
* 10-PIO-INTERRUPCAO
*
* [ref] http://www.atmel.com/Images/Atmel-42142-SAM-AT03258-Using-Low-Power-Mode-in-SAM4E-Microcontroller_Application-Note.pdf
* [ref] https://www.eecs.umich.edu/courses/eecs373/labs/refs/M3%20Guide.pdf
************************************************************************/


#include "asf.h"
#include "conf_clock.h"



/************************************************************************/
/* Defines                                                              */
/************************************************************************/

/**
 * LEDs
 */
#define LED_PIO_ID		ID_PIOC
#define LED_PIO         PIOC
#define LED_PIN		    8
#define LED_PIN_MASK    (1<<LED_PIN)

#define LED1_PIO_ID	 ID_PIOC
#define LED1_PIO			 PIOC
#define LED1_PIN			 19
#define LED1_PIN_MASK (1<<LED1_PIN)

#define LED2_PIO_ID	 ID_PIOD
#define LED2_PIO			 PIOD
#define LED2_PIN			 26
#define LED2_PIN_MASK (1<<LED2_PIN)

#define LED3_PIO_ID	 ID_PIOD
#define LED3_PIO			 PIOD
#define LED3_PIN			 11
#define LED3_PIN_MASK (1<<LED3_PIN)

/**
 * Botão
 */
#define BUT_PIO_ID      ID_PIOA
#define BUT_PIO         PIOA
#define BUT_PIN		    11
#define BUT_PIN_MASK    (1 << BUT_PIN)
#define BUT_DEBOUNCING_VALUE  79

#define BUT1_PIO_ID      ID_PIOA
#define BUT1_PIO         PIOA
#define BUT1_PIN		    2
#define BUT1_PIN_MASK    (1 << BUT1_PIN)
#define BUT1_DEBOUNCING_VALUE  79

#define BUT2_PIO_ID      ID_PIOD
#define BUT2_PIO         PIOD
#define BUT2_PIN		 30
#define BUT2_PIN_MASK    (1 << BUT2_PIN)
#define BUT2_DEBOUNCING_VALUE  79

#define BUT3_PIO_ID      ID_PIOC
#define BUT3_PIO         PIOC
#define BUT3_PIN		 13
#define BUT3_PIN_MASK    (1 << BUT3_PIN)
#define BUT3_DEBOUNCING_VALUE  79

/************************************************************************/
/* prototype                                                             */
/************************************************************************/
void led_init(int estado);
void led1_init(int estado);
void led2_init(int estado);
void led3_init(int estado);
void but_init(void);
void but1_init(void);
void but2_init(void);
void but3_init(void);
void but_Handler();
void but1_Handler();
void but2_Handler();
void but3_Handler();

/************************************************************************/
/* Interrupçcões                                                        */
/************************************************************************/

void but1_Handler(){
    uint32_t pioIntStatus;
    pioIntStatus =  pio_get_interrupt_status(BUT1_PIO);
    
   if(pio_get_output_data_status(LED1_PIO, LED1_PIN_MASK))
    pio_clear(LED1_PIO, LED1_PIN_MASK);
   else
    pio_set(LED1_PIO,LED1_PIN_MASK);    
}

void but2_Handler(){
    uint32_t pioIntStatus;
    pioIntStatus =  pio_get_interrupt_status(BUT2_PIO);
    
   if(pio_get_output_data_status(LED2_PIO, LED2_PIN_MASK))
    pio_clear(LED2_PIO, LED2_PIN_MASK);
   else
    pio_set(LED2_PIO,LED2_PIN_MASK);    
}

void but3_Handler(){
    uint32_t pioIntStatus;
    pioIntStatus =  pio_get_interrupt_status(BUT3_PIO);
	
   if(pio_get_output_data_status(LED3_PIO, LED3_PIN_MASK))
    pio_clear(LED3_PIO, LED3_PIN_MASK);
   else
    pio_set(LED3_PIO,LED3_PIN_MASK);    
}

/************************************************************************/
/* Funções	                                                            */
/************************************************************************/

/**
 * @Brief Inicializa o pino do LED
 */
void led_init(int estado){
    pmc_enable_periph_clk(LED_PIO_ID);
    pio_set_output(LED_PIO, LED_PIN_MASK, 1, 0, 0 );
};

void led1_init(int estado){
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_set_output(LED1_PIO, LED1_PIN_MASK, 1, 0, 0 );
};

void led2_init(int estado){
	pmc_enable_periph_clk(LED2_PIO_ID);
	pio_set_output(LED2_PIO, LED2_PIN_MASK, 1, 0, 0 );
};

void led3_init(int estado){
	pmc_enable_periph_clk(LED3_PIO_ID);
	pio_set_output(LED3_PIO, LED3_PIN_MASK, 1, 0, 0 );
};

/**
 * @Brief Inicializa o pino do BUT
 *  config. botao em modo entrada enquanto 
 *  ativa e configura sua interrupcao.
 */
void but_init(void){
    /* config. pino botao em modo de entrada */
    pmc_enable_periph_clk(BUT_PIO_ID);
    pio_set_input(BUT_PIO, BUT_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
    
    /* config. interrupcao em borda de descida no botao do kit */
    /* indica funcao (but_Handler) a ser chamada quando houver uma interrupção */
    pio_enable_interrupt(BUT_PIO, BUT_PIN_MASK);
    //pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIN_MASK, PIO_IT_FALL_EDGE, but_Handler);
    
    /* habilita interrupçcão do PIO que controla o botao */
    /* e configura sua prioridade                        */
    NVIC_EnableIRQ(BUT_PIO_ID);
    NVIC_SetPriority(BUT_PIO_ID, 1);
};

void but1_init(void){
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pio_set_input(BUT1_PIO, BUT1_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_enable_interrupt(BUT1_PIO, BUT1_PIN_MASK);
	pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIN_MASK, PIO_IT_RISE_EDGE, but1_Handler);
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 1);
};

void but2_init(void){
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pio_set_input(BUT2_PIO, BUT2_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_enable_interrupt(BUT2_PIO, BUT2_PIN_MASK);
	pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIN_MASK, PIO_IT_FALL_EDGE, but2_Handler);
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, 1);
};

void but3_init(void){
	pmc_enable_periph_clk(BUT3_PIO_ID);
	pio_set_input(BUT3_PIO, BUT3_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_enable_interrupt(BUT3_PIO, BUT3_PIN_MASK);
	pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_PIN_MASK, PIO_IT_FALL_EDGE && PIO_IT_FALL_EDGE, but3_Handler);
	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT3_PIO_ID, 1);
};


/************************************************************************/
/* Main                                                                 */
/************************************************************************/
int main(void)
{
	/************************************************************************/
	/* Inicialização básica do uC                                           */
	/************************************************************************/
	sysclk_init();
	WDT->WDT_MR = WDT_MR_WDDIS;

	/************************************************************************/
	/* Inicializao I/OS                                                     */
	/************************************************************************/
	led_init(1);
	led1_init(1);
	led2_init(1);
	led3_init(1);	
    but_init();
	but1_init();
	but2_init();
	but3_init();

	/************************************************************************/
	/* Super loop                                                           */
	/************************************************************************/
	while(1){
       /* entra em modo sleep */
       pmc_sleep(SLEEPMGR_SLEEP_WFI);
	   
	   for(int i=0; i<5; i++){
		pio_set(LED_PIO,LED_PIN_MASK);
		delay_ms(60);
		pio_clear(LED_PIO, LED_PIN_MASK);
	   }
	  	   
	   
	   
	};
}


