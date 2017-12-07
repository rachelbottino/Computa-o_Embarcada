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
#define SENSOR_PIN			26
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
// void ledConfig();
// void butConfig();
// void sensor_leitura();

/************************************************************************/
/* Funções	                                                            */
/************************************************************************/
/**
 * @Brief Inicializa o pino do LED
 */
void ledverdeConfig(){
	//Representa o status de irrigação ATIVO
	PMC->PMC_PCER0 = (1<<LED_VERDE_PIO_ID);	
	LED_VERDE_PIO->PIO_OER  = LED_VERDE_PIN_MASK;
	LED_VERDE_PIO->PIO_PER  = LED_VERDE_PIN_MASK;
	LED_VERDE_PIO->PIO_CODR = LED_VERDE_PIN_MASK;
};

void ledvermelhoConfig(){
	//Representa o status de irrigação INATIVO
	PMC->PMC_PCER0 = (1<<LED_VERMELHO_PIO_ID);
	LED_VERMELHO_PIO->PIO_OER  = LED_VERMELHO_PIN_MASK;
	LED_VERMELHO_PIO->PIO_PER  = LED_VERMELHO_PIN_MASK;
	LED_VERMELHO_PIO->PIO_SODR = LED_VERMELHO_PIN_MASK;
};

void butConfig(){
	PMC->PMC_PCER0= (1<<BUT_PIO_ID);
	BUT_PIO->PIO_PER = BUT_PIN_MASK;
	BUT_PIO->PIO_ODR = BUT_PIN_MASK;
	BUT_PIO->PIO_PUER= BUT_PIN_MASK;
	BUT_PIO->PIO_IFER= BUT_PIN_MASK;
};

void sensorConfig(){
	PMC->PMC_PCER0= (1<<SENSOR_PIO_ID);
	SENSOR_PIO->PIO_ODR = SENSOR_PIN_MASK;	
}

void valvulaConfig(){
	//Representada pelo LED azul na placa
	PMC->PMC_PCER0 = (1<<VALVULA_PIO_ID);
	VALVULA_PIO->PIO_OER  = VALVULA_PIN_MASK;
	VALVULA_PIO->PIO_PER  = VALVULA_PIN_MASK;
	VALVULA_PIO->PIO_CODR = VALVULA_PIN_MASK;
};

void offIrrigacao(){
	LED_VERDE_PIO->PIO_SODR = LED_VERDE_PIN_MASK; //apaga led verde (status: INATIVO)
	LED_VERMELHO_PIO->PIO_CODR = LED_VERMELHO_PIN_MASK; //acende led vermelho (status: INATIVO)
	VALVULA_PIO->PIO_SODR = VALVULA_PIN_MASK;	//apaga led azul (valvula DESLIGADA)
};

void onIrrigacao(){
	LED_VERDE_PIO->PIO_CODR = LED_VERDE_PIN_MASK; //acende led verde	(status: ATIVO)
	LED_VERMELHO_PIO->PIO_SODR = LED_VERMELHO_PIN_MASK; //apaga led vermelho (status: ATIVO)
	VALVULA_PIO->PIO_CODR = VALVULA_PIN_MASK; //acende led azul (valvula DESLIGADA)
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
	while(1){		
		//leitura do botao
		
 		if( !(BUT_PIO->PIO_PDSR & BUT_PIN_MASK)  ){
 			onIrrigacao();
 		}
 			
 		//leitura sensor
 		else if ( (SENSOR_PIO->PIO_PDSR & SENSOR_PIN_MASK) ){
 			onIrrigacao();
 		}
 
 		else{
 			offIrrigacao();
 		}
 		
 		delay_ms(100);
 	};
}