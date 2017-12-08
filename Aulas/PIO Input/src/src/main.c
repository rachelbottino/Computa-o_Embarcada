/**************************************************************************
* Rafael Corsi   - Insper
* rafael.corsi@insper.edu.br
*
* Computação Embarcada
*
* 08-PIO-ENTRADA
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
#define LED_PIO       PIOC
#define LED_PIN			  8
#define LED_PIN_MASK	(1<<LED_PIN)

/**
 * Botão
 */
/**
 * Botão
 */
#define BUT_PIO_ID      ID_PIOA
#define BUT_PIO         PIOA
#define BUT_PIN		    11
#define BUT_PIN_MASK    (1 << BUT_PIN)
#define BUT_DEBOUNCING_VALUE  79


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

/** pisca led status **/
bool pisca_status = false;
uint32_t pisca_delay = 500;

/************************************************************************/
/* Prototipação                                                        */
/************************************************************************/
//void ledConfig();

/************************************************************************/
/* Funções	                                                            */
/************************************************************************/

/**
 * @Brief Inicializa o pino do LED
 */
void ledConfig(int estado){
	PMC->PMC_PCER0    = (1<<LED_PIO_ID);	    // Ativa clock do periférico no PMC
	LED_PIO->PIO_PER  = LED_PIN_MASK;           // Ativa controle do pino no PIO    (PIO ENABLE register)
	LED_PIO->PIO_OER  = LED_PIN_MASK;           // Ativa saída                      (Output ENABLE register)
  if(!estado)                                 // Checa pela inicialização desejada
    LED_PIO->PIO_CODR = LED_PIN_MASK;       // Coloca 0 na saída                (CLEAR Output Data register)
  else
    LED_PIO->PIO_SODR = LED_PIN_MASK;       // Coloca 1 na saída                (SET Output Data register)
};

void butConfig(Pio *but_pio, const uint32_t but_mask,  const uint32_t but_id){
	// Configura botao
	PMC->PMC_PCER0      = (1<<but_id);     // Ativa clock do periférico no PMC
	but_pio->PIO_ODR	  = but_mask;        // Desativa saída                   (Output DISABLE register)
	but_pio->PIO_PER	  = but_mask;        // Ativa controle do pino no PIO    (PIO ENABLE register)
	but_pio->PIO_PUER	  = but_mask;        // Ativa pull-up no PIO             (PullUp ENABLE register)
	but_pio->PIO_IFER	  = but_mask;        // Ativa debouncing
	but_pio->PIO_IFSCER = but_mask;        // Ativa clock periferico
}


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
	/* Inicializa perifericos                                               */
	/************************************************************************/
	// Configura LED em modo saída
	ledConfig(1);
	butConfig(BUT_PIO, BUT_PIN_MASK, BUT_PIO_ID);
	butConfig(BUT1_PIO, BUT1_PIN_MASK, BUT1_PIO_ID);
	butConfig(BUT3_PIO, BUT3_PIN_MASK, BUT3_PIO_ID);
	
	/************************************************************************/
	/* Super loop                                                           */
	/************************************************************************/
	while(1){
		/**
     * @Brief Verifica constantemente o status do botão
     * 1 : não apertado
     * 0 : apertado
     */
		
		if(!(BUT_PIO->PIO_PDSR & BUT_PIN_MASK)){
			//mudando status do botao
			pisca_status = !pisca_status;
		}
		if(!(BUT1_PIO->PIO_PDSR & BUT1_PIN_MASK)){
			pisca_delay += 10;
		}
		if(!(BUT3_PIO->PIO_PDSR & BUT3_PIN_MASK)){
			pisca_delay -= 10;
		}		
		if(pisca_status){
			pio_set(LED_PIO,LED_PIN_MASK);
			delay_ms(60);
			pio_clear(LED_PIO, LED_PIN_MASK); 
		}
	};
}
