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
	ledConfig();

	// Configura botao
	PMC->PMC_PCER0      = (1<<BUT_PIO_ID);     // Ativa clock do periférico no PMC
	BUT_PIO->PIO_ODR	  = BUT_PIN_MASK;        // Desativa saída                   (Output DISABLE register)
	BUT_PIO->PIO_PER	  = BUT_PIN_MASK;        // Ativa controle do pino no PIO    (PIO ENABLE register)
	BUT_PIO->PIO_PUER	  = BUT_PIN_MASK;        // Ativa pull-up no PIO             (PullUp ENABLE register)
	BUT_PIO->PIO_IFER	  = BUT_PIN_MASK;        // Ativa debouncing
	BUT_PIO->PIO_IFSCER = BUT_PIN_MASK;        // Ativa clock periferico
	BUT_PIO->PIO_SCDR	  = BUT_DEBOUNCING_VALUE;// Configura a frequencia do debouncing

	/************************************************************************/
	/* Super loop                                                           */
	/************************************************************************/
	while(1){
		/**
     * @Brief Verifica constantemente o status do botão
     * 1 : não apertado
     * 0 : apertado
     */
    if(BUT_PIO->PIO_PDSR & (BUT_PIN_MASK)){
			LED_PIO->PIO_CODR = LED_PIN_MASK;
    }
		else{
			LED_PIO->PIO_SODR = LED_PIN_MASK;
    }
	};
}
