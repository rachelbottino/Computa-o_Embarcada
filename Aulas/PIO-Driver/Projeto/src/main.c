/************************************************************************
* Rafael Corsi   - Insper
* rafael.corsi@insper.edu.br
*
* Computação Embarcada
*
* 09-PIO-DRIVER
************************************************************************/


#include "asf.h"
#include "conf_clock.h"
#include "Driver/pmc_insper.h"
#include "Driver/pio_insper.h"

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

/**
 * Botão
 */
#define BUT_PIO_ID      ID_PIOA
#define BUT_PIO         PIOA
#define BUT_PIN		    11
#define BUT_PIN_MASK    (1 << BUT_PIN)
#define BUT_DEBOUNCING_VALUE  79

/************************************************************************/
/* prototype                                                             */
/************************************************************************/
void led_init(int estado);
void but_init(void);

/************************************************************************/
/* Funções	                                                            */
/************************************************************************/

/**
 * @Brief Inicializa o pino do LED
 */
void led_init(int estado){
	_pmc_enable_periph_clock(LED_PIO_ID);	// Ativa clock do periférico no PMC
	_pio_set_output(LED_PIO, LED_PIN_MASK,estado,0); // Ativa controle do pino no PIO    (PIO ENABLE register) e Ativa saída                      (Output ENABLE register)
	
	if(!estado){                              // Checa pela inicialização desejada
		_pio_clear(LED_PIO, LED_PIN_MASK);      // Coloca 0 na saída                (CLEAR Output Data register)
	}
	else {
		_pio_set(LED_PIO, LED_PIN_MASK);       // Coloca 1 na saída                (SET Output Data register)
	}
};

/**
 * @Brief Inicializa o pino do BUT
 */
void but_init(void){
	_pmc_enable_periph_clock(ID_PIOA);		// Ativa clock do periférico no PMC	
	_pio_pull_up(BUT_PIO,BUT_PIN_MASK,1);   // Ativa pull-up no PIO             (PullUp ENABLE register)
	_pio_set_input(BUT_PIO,BUT_PIN_MASK,1); // Desativa saída                   (Output DISABLE register)
											// Ativa controle do pino no PIO    (PIO ENABLE register)
											// Ativa debouncing
											// Ativa clock periferico
        
	BUT_PIO->PIO_SCDR	 = BUT_DEBOUNCING_VALUE;// Configura a frequencia do debouncing
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
    but_init();

	/************************************************************************/
	/* Super loop                                                           */
	/************************************************************************/

	while(1){
		/**
		* @Brief Verifica constantemente o status do botão
		* 1 : não apertado
		* 0 : apertado
		*/
	    if(_pio_get_output_data_status(BUT_PIO, BUT_PIN_MASK)){
			_pio_clear(LED_PIO, LED_PIN_MASK);
        }
		else{
			_pio_set(LED_PIO, LED_PIN_MASK);
        }
	};
}


