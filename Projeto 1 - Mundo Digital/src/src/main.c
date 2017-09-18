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
#define BLINK_PERIOD     1000
int blink = BLINK_PERIOD;
/**
 * LEDs
 */
#define LED_VERDE_PIO_ID		ID_PIOC
#define LED_VERDE_PIO         PIOC
#define LED_VERDE_PIN			13
#define LED_VERDE_PIN_MASK	(1<<LED_VERDE_PIN) 

#define LED_VERMELHO_PIO_ID		ID_PIOD
#define LED_VERMELHO_PIO         PIOD
#define LED_VERMELHO_PIN		30
#define LED_VERMELHO_PIN_MASK	(1<<LED_VERMELHO_PIN)

/**
 * Botão
 */ 
#define BUT_PIO_ID		ID_PIOA
#define BUT_PIO         PIOA
#define BUT_PIN			6
#define BUT_PIN_MASK	(1 << BUT_PIN)

/**
/* Sensor de umidade de solo  /*                                                                  
*/
#define SENSOR_PIO_ID		ID_PIOD
#define SENSOR_PIO         PIOD
#define SENSOR_PIN			22
#define SENSOR_PIN_MASK	(1 << SENSOR_PIN)

/**
/* Válvula solenoide /*
*/
#define VALVULA_PIO_ID		ID_PIOD
#define VALVULA_PIO         PIOD
#define VALVULA_PIN			11
#define VALVULA_PIN_MASK	(1 << VALVULA_PIN)


/************************************************************************/
/* Prototipação                                                        */
/************************************************************************/
//void ledConfig();
//void butConfig();
//void sensor_leitura();

/************************************************************************/
/* Funções	                                                            */
/************************************************************************/
/**
 * @Brief Inicializa o pino do LED
 */
void ledverdeConfig(){
	PMC->PMC_PCER0 = (1<<LED_VERDE_PIO_ID);	
	PIOC->PIO_OER  = (1 << LED_VERDE_PIN);
	PIOC->PIO_PER  = (1 << LED_VERDE_PIN);
	PIOC->PIO_CODR = (1 << LED_VERDE_PIN);
};

void ledvermelhoConfig(){
	PMC->PMC_PCER0 = (1<<LED_VERMELHO_PIO_ID);
	PIOD->PIO_OER  = (1 << LED_VERMELHO_PIN);
	PIOD->PIO_PER  = (1 << LED_VERMELHO_PIN);
	PIOD->PIO_CODR = (1 << LED_VERMELHO_PIN);
};

void butConfig(){
	PMC->PMC_PCER0= (1<<BUT_PIO_ID);
	PIOA->PIO_PER = (1<<BUT_PIN);
	PIOA->PIO_ODR = (1<<BUT_PIN);
	PIOA->PIO_PUER= (1<<BUT_PIN);
	PIOA->PIO_IFER= (1<<BUT_PIN);
};

void sensorConfig(){
	PMC->PMC_PCER0= (1<<SENSOR_PIO_ID);
	PIOD->PIO_ODR = (1<<SENSOR_PIN);	
}

void valvulaConfig(){
	PMC->PMC_PCER0 = (1<<VALVULA_PIO_ID);
	PIOD->PIO_OER  = VALVULA_PIN_MASK;
	PIOD->PIO_PER  = VALVULA_PIN_MASK;
	PIOD->PIO_CODR = VALVULA_PIN_MASK;
};

void offIrrigacao(){
	PIOC->PIO_SODR = (1 << LED_VERDE_PIN);
	PIOD->PIO_CODR = (1 << LED_VERMELHO_PIN);
	PIOD->PIO_CODR = VALVULA_PIN_MASK;	
};

void onIrrigacao(){
	PIOC->PIO_CODR = (1 << LED_VERDE_PIN);
	PIOD->PIO_SODR = (1 << LED_VERMELHO_PIN);
	PIOD->PIO_SODR = VALVULA_PIN_MASK;
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
	board_init();
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	/************************************************************************/
	/* Inicializa perifericos                                               */
	/************************************************************************/
	// Configura LED em modo saída
	ledverdeConfig();
	ledvermelhoConfig();

	// Configura botao
	butConfig();	
	
	//Configura Sensor
	sensorConfig();
	
	//Configura valvula
	valvulaConfig();
	
	//Configura irrigação
	offIrrigacao();
		
	/************************************************************************/
	/* Super loop                                                           */
	/************************************************************************/
	volatile int botao;
	while(1){		
		//leitura do sensor
		
		if( !(BUT_PIO->PIO_PDSR & BUT_PIN_MASK)  ){		
			onIrrigacao();
		}
			
		//leitura botao
		else if ( (PIOD->PIO_PDSR & SENSOR_PIN_MASK) ){
				onIrrigacao();
		}

		else{
			offIrrigacao();
		}
		
		delay_ms(50);
	};
}