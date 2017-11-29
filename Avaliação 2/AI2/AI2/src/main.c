#include "asf.h"
#include "image.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "ioport.h"
#include "logo.h"

/** ASCII char definition for backspace. */
#define ASCII_BS    0x7F
/** ASCII char definition for carriage return. */
#define ASCII_CR    13

#define STRING_EOL    "\r\n"
#define STRING_HEADER "-- SAME70 LCD DEMO --"STRING_EOL	\
	"-- "BOARD_NAME " --"STRING_EOL	\
	"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL
	
/** Reference voltage for AFEC,in mv. */
#define VOLT_REF        (3300)

/** The maximal digital value */
/** 2^12 - 1                  */
#define MAX_DIGITAL     (4095UL)
	
/** The conversion data is done flag */
volatile bool is_conversion_done = false;

/** The conversion data value */
volatile uint32_t g_ul_value = 0;

/* Canal do sensor de temperatura */
#define AFEC_CHANNEL_TEMP_SENSOR 11

 /* buffer para recebimento de temperatura */
 uint8_t buffer[100];
 
 /* buffer para recebimento de horario */
 uint8_t bufferTime[100];
 
 /*temperatura*/
 uint8_t temp; 
 
 /*largura grafico*/
 uint8_t largura;
 
 /************************************************************************/
/* DEFINES                                                              */
/************************************************************************/

#define YEAR        2017
#define MOUNTH      11
#define DAY         29
#define WEEK        48
#define HOUR        17
#define MINUTE      47
#define SECOND      0

 /* Horario */
 uint32_t hora;
 uint32_t minuto;
 uint32_t segundo;

struct ili9488_opt_t g_ili9488_display_opt;

/**
}
/**
 * \brief Configure UART console.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate =		CONF_UART_BAUDRATE,
		.charlength =	CONF_UART_CHAR_LENGTH,
		.paritytype =	CONF_UART_PARITY,
		.stopbits =		CONF_UART_STOP_BITS,
	};

	/* Configure UART console. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

static int32_t convert_adc_to_temp(int32_t ADC_value){
  
  int32_t ul_vol;
  int32_t ul_temp;
  
	ul_vol = ADC_value * VOLT_REF / MAX_DIGITAL;

  /*
   * According to datasheet, The output voltage VT = 0.72V at 27C
   * and the temperature slope dVT/dT = 2.33 mV/C
   */
  ul_temp = (ul_vol - 720)  * 100 / 233 + 27;
  return(ul_temp);
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

/************************************************************************/
/* Call backs / Handler                                                 */
/************************************************************************/


static void AFEC_Temp_callback(void)
{
	g_ul_value = afec_channel_get_value(AFEC0, AFEC_CHANNEL_TEMP_SENSOR);
	is_conversion_done = true;
}

/**
 * \brief Main application function.
 *
 * Initialize system, UART console, network then start weather client.
 *
 * \return Program return value.
 */
int main(void)
{
	/* Initialize the board. */
	sysclk_init();
	board_init();
	ioport_init();
	
	/* inicializa delay */
	delay_init(sysclk_get_cpu_hz());
	
	/* Initialize the UART console. */
	configure_console();
	printf(STRING_HEADER);
	
	/** Inicializa RTC */
	RTC_init();

	/* Set direction and pullup on the given button IOPORT */
	ioport_set_pin_dir(GPIO_PUSH_BUTTON_1, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(GPIO_PUSH_BUTTON_1, IOPORT_MODE_PULLUP);

	/* Initialize display parameter */
	g_ili9488_display_opt.ul_width = ILI9488_LCD_WIDTH;
	g_ili9488_display_opt.ul_height = ILI9488_LCD_HEIGHT;
	g_ili9488_display_opt.foreground_color = COLOR_CONVERT(COLOR_WHITE);
	g_ili9488_display_opt.background_color = COLOR_CONVERT(COLOR_WHITE);

	/* Initialize LCD */
	ili9488_init(&g_ili9488_display_opt);
	ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_TURQUOISE));
	ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, 120-1);
	ili9488_draw_filled_rectangle(0, 360, ILI9488_LCD_WIDTH-1, 480-1);
	//ili9488_draw_pixmap(0, 50, 319, 129, logoImage);

    /* Escreve na tela */
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
	ili9488_draw_filled_rectangle(0, 300, ILI9488_LCD_WIDTH-1, 315);
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
	ili9488_draw_string(30, 100, (uint8_t *)"Sensor de Temperatura");
	
	 /************************************* 
   * Ativa e configura AFEC
   *************************************/  

  /* Ativa AFEC - 0 */
	afec_enable(AFEC0);

  /* struct de configuracao do AFEC */
	struct afec_config afec_cfg;

  /* Carrega parametros padrao */
	afec_get_config_defaults(&afec_cfg);

  /* Configura AFEC */
	afec_init(AFEC0, &afec_cfg);
  
  /* Configura trigger por software */
	afec_set_trigger(AFEC0, AFEC_TRIG_SW);
  
  /* configura call back */
 	afec_set_callback(AFEC0, AFEC_INTERRUPT_EOC_11,	AFEC_Temp_callback, 1); 
   
  /*** Configuracao específica do canal AFEC ***/
	struct afec_ch_config afec_ch_cfg;
	afec_ch_get_config_defaults(&afec_ch_cfg);
	afec_ch_cfg.gain = AFEC_GAINVALUE_0;
	afec_ch_set_config(AFEC0, AFEC_CHANNEL_TEMP_SENSOR, &afec_ch_cfg);
  
  /*
   * Calibracao:
	 * Because the internal ADC offset is 0x200, it should cancel it and shift
	 * down to 0.
	 */
	afec_channel_set_analog_offset(AFEC0, AFEC_CHANNEL_TEMP_SENSOR, 0x200);

  /***  Configura sensor de temperatura ***/
	struct afec_temp_sensor_config afec_temp_sensor_cfg;

	afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
	afec_temp_sensor_set_config(AFEC0, &afec_temp_sensor_cfg);

  /* Selecina canal e inicializa conversão */  
	afec_channel_enable(AFEC0, AFEC_CHANNEL_TEMP_SENSOR);
	afec_start_software_conversion(AFEC0);
  
	while (1) {
		if(is_conversion_done == true) {
			is_conversion_done = false;	
			temp = convert_adc_to_temp(g_ul_value);			
		
			printf("Temperatura : %d  \r\n",temp);
			afec_start_software_conversion(AFEC0);
			
			//limpa a tela escrevendo um retangulo branco
			ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
			ili9488_draw_filled_rectangle(0, 140, ILI9488_LCD_WIDTH-1,360);
			//delay_s(1);
			
			//escreve temperatura
			ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
			ili9488_draw_string(30, 150, (uint8_t *)buffer);
			
			sprintf(buffer, "Temperatura : %d  \r\n",temp);

			
			//determina a cor do grafico conforme temperatura
			if(temp<25){
				ili9488_set_foreground_color(COLOR_CONVERT(COLOR_LIGHTBLUE));
				printf("||| \n");
				ili9488_draw_filled_rectangle(10, 180, 60,200);
			}
			else if(temp<28){
				ili9488_set_foreground_color(COLOR_CONVERT(COLOR_YELLOW));
				printf("|||||| \n");
				ili9488_draw_filled_rectangle(10, 180, 120,200);
			}
			else if(temp<31){
				ili9488_set_foreground_color(COLOR_CONVERT(COLOR_ORANGE));
				printf("|||||||||\n");
				ili9488_draw_filled_rectangle(10, 180, 180,200);
			}
			else if(temp<34){
				ili9488_set_foreground_color(COLOR_CONVERT(COLOR_TOMATO));
				printf("||||||||||||\n");
				ili9488_draw_filled_rectangle(10, 180, 240,200);
			}
			else{
				ili9488_set_foreground_color(COLOR_CONVERT(COLOR_RED));
				printf("|||||||||||||||\n");
				ili9488_draw_filled_rectangle(10, 180, 300,200);
			}
			
			//para temperaturas de 20 a 40 graus, regra de tres para o grafico
			//largura = ((temp*300)/32)-168;
			
			//escreve retangulo
			//ili9488_draw_filled_rectangle(10, 180, largura,200);
			
			//horario
			rtc_get_time(RTC, &hora, &minuto, &segundo);
			sprintf(bufferTime, "Hora atual: %d:%d:%d \n", hora,minuto,segundo);
			ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
			ili9488_draw_string(30, 230, (uint8_t *)bufferTime);

			
			delay_s(1);
		}
	
	}
	return 0;
}
