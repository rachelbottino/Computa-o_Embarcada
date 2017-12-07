#include "asf.h"
#include "main.h"
#include <string.h>
#include "bsp/include/nm_bsp.h"
#include "driver/include/m2m_wifi.h"
#include "socket/include/socket.h"

#define STRING_EOL    "\r\n"
#define STRING_HEADER "-- WINC1500 weather client example --"STRING_EOL	\
	"-- "BOARD_NAME " --"STRING_EOL	\
	"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL

/** IP address of host. */
uint32_t gu32HostIp = 0;

/** TCP client socket handlers. */
static SOCKET tcp_client_socket = -1;

/** Receive buffer definition. */
static uint8_t gau8ReceivedBuffer[MAIN_WIFI_M2M_BUFFER_SIZE] = {0};
static uint8_t gau8PostBuffer[MAIN_WIFI_M2M_BUFFER_SIZE] = {0};

/** Wi-Fi status variable. */
static bool gbConnectedWifi = false;

/** Get host IP status variable. */
/** Wi-Fi connection state */
static uint8_t wifi_connected;


/** Instance of HTTP client module. */
static bool gbHostIpByName = false;

/** TCP Connection status variable. */
static bool gbTcpConnection = false;

/** Server host name. */
static char server_host_name[] = MAIN_SERVER_NAME;

uint8_t recvOk = false;
uint8_t socketConnected = false;

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
 * Bot�o
 */
#define BUT_PIO_ID      ID_PIOA
#define BUT_PIO         PIOA
#define BUT_PIN		    11
#define BUT_PIN_MASK    (1 << BUT_PIN)
#define BUT_DEBOUNCING_VALUE  79

/**
 * Bot�o
 */ 
#define BUT_EXT_PIO_ID		ID_PIOA
#define BUT_EXT_PIO         PIOA
#define BUT_EXT_PIN			6
#define BUT_EXT_PIN_MASK	(1 << BUT_PIN)

/**
/* Sensor de umidade de solo  /*
*/
#define SENSOR_PIO_ID		ID_PIOD
#define SENSOR_PIO         PIOD
#define SENSOR_PIN			26
#define SENSOR_PIN_MASK	(1 << SENSOR_PIN)

/**
/* V�lvula solenoide /*
*/
#define VALVULA_PIO_ID		ID_PIOD
#define VALVULA_PIO         PIOD
#define VALVULA_PIN			11
#define VALVULA_PIN_MASK	(1 << VALVULA_PIN)

/** status da irriga��o **/
bool irr_status = false;

bool sensor_status = false; 


/************************************************************************/
/* Fun�oes                                                             */
/************************************************************************/
/**
 * @Brief Inicializa o pino do LED
 */
void ledverdeConfig(){
	//Representa o status de irriga��o ATIVO
	PMC->PMC_PCER0 = (1<<LED_VERDE_PIO_ID);	
	LED_VERDE_PIO->PIO_OER  = LED_VERDE_PIN_MASK;
	LED_VERDE_PIO->PIO_PER  = LED_VERDE_PIN_MASK;
	LED_VERDE_PIO->PIO_CODR = LED_VERDE_PIN_MASK;
};

void ledvermelhoConfig(){
	//Representa o status de irriga��o INATIVO
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
/* prototype                                                             */
/************************************************************************/
void but_init(void);
void but_Handler();

/************************************************************************/
/* Interrup�c�es                                                        */
/************************************************************************/
void but_Handler(){
	uint32_t pioIntStatus;
	pioIntStatus =  pio_get_interrupt_status(BUT_PIO);
	printf("sending ....\n");
	
	memset(gau8ReceivedBuffer, 0, sizeof(gau8ReceivedBuffer));
	sprintf((char *)gau8ReceivedBuffer, "%s", MAIN_PREFIX_BUFFER);
	printf("\n");
		
	if(socketConnected){
		//printf("POST \n");
		//sprintf(gau8PostBuffer,"POST / HTTP/1.1\r\n %s Accept: */*\r\n\r\n", 42);
		//sprintf(gau8PostBuffer,"POST / HTTP/1.1\r\n { status : '' } Accept: */*\r\n\r\n", "on");
			
		//printf(gau8PostBuffer);
		//send(tcp_client_socket, gau8PostBuffer, strlen((char *)gau8PostBuffer), 0);		
		

		printf("send \n");
		send(tcp_client_socket, gau8ReceivedBuffer, strlen((char *)gau8ReceivedBuffer), 0);
		memset(gau8ReceivedBuffer, 0, MAIN_WIFI_M2M_BUFFER_SIZE);
		recv(tcp_client_socket, &gau8ReceivedBuffer[0], MAIN_WIFI_M2M_BUFFER_SIZE, 0);
		recvOk = false;

	}
}

void but_init(void){
	/* config. pino botao em modo de entrada */
	pmc_enable_periph_clk(BUT_PIO_ID);
	pio_set_input(BUT_PIO, BUT_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_enable_interrupt(BUT_PIO, BUT_PIN_MASK);     
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIN_MASK, PIO_IT_RISE_EDGE, but_Handler);	                  
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 1);
};

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


/* 
 * Check whether "cp" is a valid ascii representation
 * of an Internet address and convert to a binary address.
 * Returns 1 if the address is valid, 0 if not.
 * This replaces inet_addr, the return value from which
 * cannot distinguish between failure and a local broadcast address.
 */
 /* http://www.cs.cmu.edu/afs/cs/academic/class/15213-f00/unpv12e/libfree/inet_aton.c */
int inet_aton(const char *cp, in_addr *ap)
{
  int dots = 0;
  register u_long acc = 0, addr = 0;

  do {
	  register char cc = *cp;

	  switch (cc) {
	    case '0':
	    case '1':
	    case '2':
	    case '3':
	    case '4':
	    case '5':
	    case '6':
	    case '7':
	    case '8':
	    case '9':
	        acc = acc * 10 + (cc - '0');
	        break;

	    case '.':
	        if (++dots > 3) {
		    return 0;
	        }
	        /* Fall through */

	    case '\0':
	        if (acc > 255) {
		    return 0;
	        }
	        addr = addr << 8 | acc;
	        acc = 0;
	        break;

	    default:
	        return 0;
    }
  } while (*cp++) ;

  /* Normalize the address */
  if (dots < 3) {
	  addr <<= 8 * (3 - dots) ;
  }

  /* Store it if requested */
  if (ap) {
	  ap->s_addr = _htonl(addr);
  }

  return 1;    
}


/**
 * \brief Callback function of IP address.
 *
 * \param[in] hostName Domain name.
 * \param[in] hostIp Server IP.
 *
 * \return None.
 */
static void resolve_cb(uint8_t *hostName, uint32_t hostIp)
{
	gu32HostIp = hostIp;
	gbHostIpByName = true;
	printf("resolve_cb: %s IP address is %d.%d.%d.%d\r\n\r\n", hostName,
			(int)IPV4_BYTE(hostIp, 0), (int)IPV4_BYTE(hostIp, 1),
			(int)IPV4_BYTE(hostIp, 2), (int)IPV4_BYTE(hostIp, 3));
}

/**
 * \brief Callback function of TCP client socket.
 *
 * \param[in] sock socket handler.
 * \param[in] u8Msg Type of Socket notification
 * \param[in] pvMsg A structure contains notification informations.
 *
 * \return None.
 */
static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
  
	/* Check for socket event on TCP socket. */
	if (sock == tcp_client_socket) {
    
		switch (u8Msg) {
		case SOCKET_MSG_CONNECT:
		{
			printf("socket_msg_connect\n"); 
			if (gbTcpConnection) {
			

				tstrSocketConnectMsg *pstrConnect = (tstrSocketConnectMsg *)pvMsg;
				if (pstrConnect && pstrConnect->s8Error >= SOCK_ERR_NO_ERROR) {
					socketConnected = true;
				} else {
					printf("socket_cb: connect error!\r\n");
					gbTcpConnection = false;
					close(tcp_client_socket);
					tcp_client_socket = -1;
				}
			}
		}
		break;
    


		case SOCKET_MSG_RECV:
		{
			char *pcIndxPtr;
			char *pcEndPtr;

			tstrSocketRecvMsg *pstrRecv = (tstrSocketRecvMsg *)pvMsg;
			if (pstrRecv && pstrRecv->s16BufferSize > 0) {
				//printf(pstrRecv->pu8Buffer);
				
				char palavra[] = "irrigacao on";
				char *ponteiro = NULL;
				ponteiro = strstr ( pstrRecv->pu8Buffer, palavra );
				if ( ponteiro ){
					printf ("ON\n");
					irr_status = true;
					}else{
					printf ( "OFF\n");
					irr_status = false;
				}
				
				memset(gau8ReceivedBuffer, 0, sizeof(gau8ReceivedBuffer));
				if(recvOk == false){
					recv(tcp_client_socket, &gau8ReceivedBuffer[0], MAIN_WIFI_M2M_BUFFER_SIZE, 0);
					recvOk = true;
				}
			} else {
				printf("socket_cb: recv error!\r\n");
				close(tcp_client_socket);
				tcp_client_socket = -1;
			}
		}
		break;

		default:
			break;
		}
	}
}

static void set_dev_name_to_mac(uint8_t *name, uint8_t *mac_addr)
{
	/* Name must be in the format WINC1500_00:00 */
	uint16 len;

	len = m2m_strlen(name);
	if (len >= 5) {
		name[len - 1] = MAIN_HEX2ASCII((mac_addr[5] >> 0) & 0x0f);
		name[len - 2] = MAIN_HEX2ASCII((mac_addr[5] >> 4) & 0x0f);
		name[len - 4] = MAIN_HEX2ASCII((mac_addr[4] >> 0) & 0x0f);
		name[len - 5] = MAIN_HEX2ASCII((mac_addr[4] >> 4) & 0x0f);
	}
}

/**
 * \brief Callback to get the Wi-Fi status update.
 *
 * \param[in] u8MsgType Type of Wi-Fi notification.
 * \param[in] pvMsg A pointer to a buffer containing the notification parameters.
 *
 * \return None.
 */
static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
			printf("wifi_cb: M2M_WIFI_CONNECTED\r\n");
			m2m_wifi_request_dhcp_client();
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
			printf("wifi_cb: M2M_WIFI_DISCONNECTED\r\n");
			gbConnectedWifi = false;
 			wifi_connected = 0;
		}

		break;
	}

	case M2M_WIFI_REQ_DHCP_CONF:
	{
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		printf("wifi_cb: IP address is %u.%u.%u.%u\r\n",
				pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
		wifi_connected = M2M_WIFI_CONNECTED;
		
    /* Obtain the IP Address by network name */
		//gethostbyname((uint8_t *)server_host_name);
		break;
	}

	default:
	{
		break;
	}
	}
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
	tstrWifiInitParam param;
	int8_t ret;
	uint8_t mac_addr[6];
	uint8_t u8IsMacAddrValid;
	struct sockaddr_in addr_in;

	/* Initialize the board. */
	sysclk_init();
	board_init();

	/* Initialize the UART console. */
	configure_console();
	printf(STRING_HEADER);

	/* Initialize the BSP. */
	nm_bsp_init();
	
	/* Inicializa Botao */
	but_init();
	
	/************************************************************************/
	/* Inicializa perifericos                                               */
	/************************************************************************/
	// Configura LED em modo sa�da
	ledverdeConfig();
	ledvermelhoConfig();

	// Configura botao
	butConfig();
		
	//Configura Sensor
	sensorConfig();
		
	//Configura valvula
	valvulaConfig();
		
	//Configura irriga��o
	offIrrigacao();
  
	/* Initialize Wi-Fi parameters structure. */
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

	/* Initialize Wi-Fi driver with data and status callbacks. */
	param.pfAppWifiCb = wifi_cb;
	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret) {
		printf("main: m2m_wifi_init call error!(%d)\r\n", ret);
		while (1) {
		}
	}
  
	/* Initialize socket module. */
	socketInit();
  
	/* Register socket callback function. */
	registerSocketCallback(socket_cb, resolve_cb);

  /* Connect to router. */
	printf("main: connecting to WiFi AP %s...\r\n", (char *)MAIN_WLAN_SSID);
	m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);

	addr_in.sin_family = AF_INET;
	addr_in.sin_port = _htons(MAIN_SERVER_PORT);
  inet_aton(MAIN_SERVER_NAME, &addr_in.sin_addr);
  printf("Inet aton : %d", addr_in.sin_addr);

  while(1){
 		m2m_wifi_handle_events(NULL);

   	if (wifi_connected == M2M_WIFI_CONNECTED) {  
    	/* Open client socket. */
			if (tcp_client_socket < 0) {
        printf("socket init \n");
				if ((tcp_client_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
					printf("main: failed to create TCP client socket error!\r\n");
					continue;
				}

				/* Connect server */
        printf("socket connecting\n");
        
				if (connect(tcp_client_socket, (struct sockaddr *)&addr_in, sizeof(struct sockaddr_in)) != SOCK_ERR_NO_ERROR) {
					close(tcp_client_socket);
					tcp_client_socket = -1;
          printf("error\n");
				}else{
          gbTcpConnection = true;
        }
	}
	
	//leitura do botao
	
	if( !(BUT_PIO->PIO_PDSR & BUT_PIN_MASK)  ){
		printf("IRRIGANDO PELO BOTAO FISICO\n");
		onIrrigacao();
	}
	
	else if(irr_status==true){
		printf("IRRIGANDO PELO BOTAO WEB\n");
		onIrrigacao();
	}
	
	//leitura sensor
	else if ( (SENSOR_PIO->PIO_PDSR & SENSOR_PIN_MASK) ){
		printf("IRRIGANDO PELO SENSOR\n");
		onIrrigacao();
	}
	
	else{
		printf("IRRIGACAO DESLIGADA\n");
		offIrrigacao();
	}
	
	delay_ms(100);
	 
	
   }
 }


	return 0;
}
