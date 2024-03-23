/*
 * HelTec Automation(TM) WIFI_LoRa_32 factory test code, witch includ
 * follow functions:
 * 
 * - Basic OLED function test;
 * 
 * - Basic serial port test(in baud rate 115200);
 * 
 * - LED blink test;
 * 
 * - WIFI connect and scan test;
 * 
 * - LoRa Ping-Pong test (DIO0 -- GPIO26 interrup check the new incoming messages);
 * 
 * - Timer test and some other Arduino basic functions.
 *
 * by Aaron.Lee from HelTec AutoMation, ChengDu, China
 * 成都惠利特自动化科技有限公司
 * https://heltec.org
 *
 * this project also realess in GitHub:
 * https://github.com/HelTecAutomation/Heltec_ESP32
*/

#include "Arduino.h"
#include "WiFi.h"
#include "images.h"
#include "LoRaWan_APP.h"
#include <Wire.h>  
#include "HT_SSD1306Wire.h"
#include <DHT.h>                // Biblioteca DHT
#include <Adafruit_Sensor.h>

/* Defines do projeto */
#define REED          46    // pin onde o sensor magnetico esta conectado
#define DHTPIN        47    // pin onde dht esta conectado
#define WindSensor    48    // The pin location of the anemometer sensor
#define NIVEL         45    // pin onde o sensor de pressão está instalado
// #define DIRECAO       35    // pin onde o sensor de pressão está instalado

#define DIAMETRO 125       // diametro interno do balde
#define RAIO     6.25      // raio interno do balde
#define VOLUME   3.05      // volume da bascula (em cm3) (1cm3 == 1ml) (1ml == 1000mm3)

#define DHTTYPE DHT11 // DHT 11
DHT dht(DHTPIN, DHTTYPE);

/***************** Variaveis Sensores *****************/
unsigned long lastSend;

float id = 1;

// Variáveis DHT
float temperatura_lida = 0;
float umidade_lida = 0;

// Variáveis pluviometro:
int val = 0;
int old_val = 0;
volatile unsigned long REEDCOUNT = 0;
float volume_coletado;

// Variaveis Sensor Nivel
float value = 0;
float metros = 0;

// Variaveis Anemometro
volatile unsigned long Rotations = 0; // cup rotation counter used in interrupt routine
volatile unsigned long ContactBounceTime = 0; // Timer to avoid contact bounce in interrupt routine
volatile unsigned long ContactBounce = 0;

unsigned long RPM = 0;            //Rotações por minuto
float speedwind = 0;             //Velocidade do vento (km/h)
float windspeed = 0;             //Velocidade do vento (m/s)

// --- Constantes ---
const float pi = 3.14159265;     //Número de pi
int period = 5000;               //Tempo de medida(miliseconds)
int delaytime = 2000;            //Invervalo entre as amostras (miliseconds)
int radius = 120;                //Raio do anemometro(mm)

byte buffer[sizeof(float) * 6];
/***************** Variaveis Sensores *****************/
                                                                                      //
                                                                                      //
                                                                                      //
                                                                                      //
/********************************* lora  *********************************************/
#define RF_FREQUENCY                                915000000 // Hz

#define TX_OUTPUT_POWER                             8        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             4         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 50 // Define the payload size here

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

typedef enum
{
    LOWPOWER,
    STATE_RX,
    STATE_TX
}States_t;

int16_t txNumber;
int16_t rxNumber;
States_t state;
bool sleepMode = false;
int16_t Rssi,rxSize;

String rssi = "RSSI --";
String packSize = "--";
String packet;
String packet_t;
String packet_h;
String packet_r;
String packet_w;
String send_num;

unsigned int Sample = 0;   // Sample number
unsigned int counter = 0;
bool receiveflag = false; // software flag for LoRa receiver, received data makes it true.
long lastSendTime = 0;        // last send time
int interval = 1000;          // interval between sends
uint64_t chipid;
int16_t RssiDetection = 0;

float temperature, humidity, rain, windSpeed, level, id_;


SSD1306Wire  factory_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst


// Término do envio de mensagens
// Muda o estado para Recebedor
void OnTxDone( void )
{
	Serial.print("TX done......");
	state=STATE_RX;

}

// Timeout no envio de mensagens
// Tenta novamente enviar a mensagens recolocando o estado de TX
void OnTxTimeout( void )
{
  Radio.Sleep( );
  Serial.print("TX Timeout......");
	state=STATE_TX;
}

// Recebimento de mensagens
// Término recebimento muda estado para Transmissor
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{

  rxNumber++;
  Rssi=rssi;
  rxSize=size;

  memcpy(rxpacket, payload, sizeof(float) );
  memcpy(&temperature, payload, sizeof(float));
  memcpy(&humidity, payload + sizeof(float), sizeof(float));
  memcpy(&rain, payload + sizeof(float) * 2, sizeof(float));
  memcpy(&windSpeed, payload + sizeof(float) * 3, sizeof(float));
  memcpy(&level, payload + sizeof(float) * 4, sizeof(float));
  memcpy(&id_, payload + sizeof(float) * 5, sizeof(float));
  
  rxpacket[size]='\0';
  Radio.Sleep( );

  Serial.printf("\r\nRssi %d , length %d\r\n",Rssi,rxSize);

  Serial.printf("\r\nTemperature: \"%.2f\"\r\n",temperature);
  Serial.printf("\r\nHumidity: \"%.2f\"\r\n",humidity);
  Serial.printf("\r\nRain: \"%.2f\"\r\n",rain);
  Serial.printf("\r\nWindSpeed: \"%.2f\"\r\n",windSpeed);
  Serial.printf("\r\nRiver Level (meters): \"%.2f\"\r\n",level);
  Serial.printf("\r\nDevice ID: \"%.2f\"\r\n",id_);

	receiveflag = true;
  state=STATE_TX;
}

void lora_init(void)
{
  Mcu.begin();
  txNumber=0;
  Rssi=0;
  rxNumber = 0;
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone = OnRxDone;

  Radio.Init( &RadioEvents );
  Radio.SetChannel( RF_FREQUENCY );
  Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                 LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                 LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                 true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

  Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                 LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                 LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                 0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
	state=STATE_RX;
}
/********************************* lora  *********************************************/
                                                                                      //
                                                                                      //
                                                                                      //
                                                                                      //



void logo(){
	factory_display.clear();
	factory_display.drawXbm(0,5,logo_width,logo_height,(const unsigned char *)logo_bits);
	factory_display.display();
}

void WIFISetUp(void)
{
	// Set WiFi to station mode and disconnect from an AP if it was previously connected
	WiFi.disconnect(true);
	delay(100);
	WiFi.mode(WIFI_STA);
	WiFi.setAutoConnect(true);
	WiFi.begin("Apê 202","COMPIUTERHOUSE");//fill in "Your WiFi SSID","Your Password"
	delay(100);

	byte count = 0;
	while(WiFi.status() != WL_CONNECTED && count < 10)
	{
		count ++;
		delay(500);
		factory_display.drawString(0, 0, "Connecting...");
		factory_display.display();
	}

	factory_display.clear();
	if(WiFi.status() == WL_CONNECTED)
	{
		factory_display.drawString(0, 0, "Connecting...OK.");
		factory_display.display();
//		delay(500);
	}
	else
	{
		factory_display.clear();
		factory_display.drawString(0, 0, "Connecting...Failed");
		factory_display.display();
		//while(1);
	}
	factory_display.drawString(0, 10, "WIFI Setup done");
	factory_display.display();
	delay(500);
}

void WIFIScan(unsigned int value)
{
	unsigned int i;
    WiFi.mode(WIFI_STA);

	for(i=0;i<value;i++)
	{
		factory_display.drawString(0, 20, "Scan start...");
		factory_display.display();

		int n = WiFi.scanNetworks();
		factory_display.drawString(0, 30, "Scan done");
		factory_display.display();
		delay(500);
		factory_display.clear();

		if (n == 0)
		{
			factory_display.clear();
			factory_display.drawString(0, 0, "no network found");
			factory_display.display();
			//while(1);
		}
		else
		{
			factory_display.drawString(0, 0, (String)n);
			factory_display.drawString(14, 0, "networks found:");
			factory_display.display();
			delay(500);

			for (int i = 0; i < n; ++i) {
			// Print SSID and RSSI for each network found
				factory_display.drawString(0, (i+1)*9,(String)(i + 1));
				factory_display.drawString(6, (i+1)*9, ":");
				factory_display.drawString(12,(i+1)*9, (String)(WiFi.SSID(i)));
				factory_display.drawString(90,(i+1)*9, " (");
				factory_display.drawString(98,(i+1)*9, (String)(WiFi.RSSI(i)));
				factory_display.drawString(114,(i+1)*9, ")");
				//factory_display.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
				delay(10);
			}
		}

		factory_display.display();
		delay(800);
		factory_display.clear();
	}
}

bool resendflag=false;
bool deepsleepflag=false;
bool interrupt_flag = false;
void interrupt_GPIO0()
{
	interrupt_flag = true;
}
void interrupt_handle(void)
{
	if(interrupt_flag)
	{
		interrupt_flag = false;
		if(digitalRead(0)==0)
		{
			if(rxNumber <=2)
			{
				resendflag=true;
			}
			else
			{
				deepsleepflag=true;
			}
		}
	}

}
void VextON(void)
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
  
}

void VextOFF(void) //Vext default OFF
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, HIGH);
}
                                                                                      //
                                                                                      //
                                                                                      //
                                                                                      //
/********************************* Sensors Functions  *********************************************/

// This is the function that the interrupt calls to increment the rotation count
void IRAM_ATTR isr_rain () {
  if ((millis() - ContactBounce) > 50 ) { // debounce the switch contact.
    REEDCOUNT = REEDCOUNT + 1;              // Adiciona 1 à cntagem de pulsos
    ContactBounce = millis();
    // Serial.println("funcao interrupcao chuva");
  }
}

void addcount(){
  counter++;
} 

// Measure wind speed
void windvelocity(){
  speedwind = 0;
  windspeed = 0;
  
  counter = 0;  
  attachInterrupt(0, addcount, RISING);
  unsigned long millis();       
  long startTime = millis();
  while(millis() < startTime + period) {
  }
}

//Função para calcular o RPM
void RPMcalc() {
  RPM = ((Rotations) * 60) / (period / 1000); // Calculate revolutions per minute (RPM)
}

//Velocidade do vento em m/s
void WindSpeed() {
  windspeed = ((4 * pi * radius * RPM) / 60) / 1000; //Calcula a velocidade do vento em m/s
} //end WindSpeed

//Velocidade do vento em km/h
void SpeedWind() {
  speedwind = (((4 * pi * radius * RPM) / 60) / 1000) * 3.6; //Calcula velocidade do vento em km/h
} //end SpeedWind

void get_temp(){
    temperatura_lida = dht.readTemperature();

    Serial.print("T: ");                    //ESCREVE O TEXTO NO DISPLAY
    Serial.println(temperatura_lida);
}

void get_umi(){
    umidade_lida = dht.readHumidity();  

    Serial.print("U: ");                    //ESCREVE O TEXTO NO DISPLAY   
    Serial.println(umidade_lida);
}

void get_rain(){
    float area_recipiente = 3.14159265 * (RAIO * RAIO); // área da seção transversal do recipiente em cm²
    float volume_por_virada = (VOLUME/area_recipiente);
    volume_coletado = (REEDCOUNT * volume_por_virada) * 10; // volume total coletado em cm³

    Serial.print("Viradas: ");
    Serial.println(REEDCOUNT);

    Serial.print("Chuva: ");
    Serial.print (volume_coletado);
    Serial.println(" mm");
}

void get_wind(){
    Sample++;
    Serial.print(Sample);
    Serial.print(": Start measurement...");
    windvelocity();
    Serial.println("   finished.");
    Serial.print("Counter: ");
    Serial.print(counter);
  
    RPMcalc();
    Serial.print("RPM: ");
    Serial.println(RPM);

    WindSpeed();
    Serial.print("WindSpeed [m/s]: ");
    Serial.println(windspeed);

    SpeedWind();
    Serial.print("WindSpeed [km/h]: ");
    Serial.println(speedwind);
}

void get_nivel() {
  value = analogRead(NIVEL);
  value = (float)value * 5 / 1023;

  Serial.printf("Tensão [V]: "); // Printa no monitor o valor lido pelo sensor
  Serial.println(val);

  metros = 0.0119*value - 0.2449; //Esse é o algoritmo que fizemos baseado nos dados que obtemos de forma "analogica", anotando os valores das tensões medidadas conforme aumentamos a profundidada do sensor, então essa função não é puramente linear
  Serial.printf("Metros:");     // Printa o valor convertido em metros
  Serial.println(metros);
}

/********************************* Sensors Functions  *********************************************/
                                                                                      //
                                                                                      //
                                                                                      //
                                                                                      //
void setup()
{
	Serial.begin(115200);
	VextON();
	delay(100);
	factory_display.init();
	factory_display.clear();
	factory_display.display();
	logo();
	delay(300);
	factory_display.clear();

	WIFISetUp();
	WiFi.disconnect(); 
	WiFi.mode(WIFI_STA);
	delay(100);

	WIFIScan(1);

	chipid=ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).
	Serial.printf("ESP32ChipID=%04X",(uint16_t)(chipid>>32));//print High 2 bytes
	Serial.printf("%08X\n",(uint32_t)chipid);//print Low 4bytes.

  /* Inicializa sensor de temperatura e umidade relativa do ar */
  dht.begin();

  lastSend = 0;

  pinMode(REED, INPUT_PULLUP);
  pinMode(WindSensor, INPUT);
  digitalWrite(WindSensor, HIGH);     //internall pull-up active
  pinMode(NIVEL, INPUT_PULLUP);
  attachInterrupt(REED, isr_rain, FALLING);

	attachInterrupt(0,interrupt_GPIO0,FALLING);
	lora_init();
	packet ="waiting lora data!";
  factory_display.drawString(0, 10, packet);
  factory_display.display();
  delay(100);
  factory_display.clear();
	pinMode(LED ,OUTPUT);
	digitalWrite(LED, LOW);  
}


void loop()
{
  interrupt_handle();
  if(deepsleepflag)
  {
    VextOFF();
    Radio.Sleep();
    SPI.end();
    pinMode(RADIO_DIO_1,ANALOG);
    pinMode(RADIO_NSS,ANALOG);
    pinMode(RADIO_RESET,ANALOG);
    pinMode(RADIO_BUSY,ANALOG);
    pinMode(LORA_CLK,ANALOG);
    pinMode(LORA_MISO,ANALOG);
    pinMode(LORA_MOSI,ANALOG);
    esp_sleep_enable_timer_wakeup(600*1000*(uint64_t)1000);
    esp_deep_sleep_start();
  }

  if(resendflag)
  {
    state = STATE_TX;
    resendflag = false;
  }

  if(receiveflag && (state==LOWPOWER) )
  {
    receiveflag = false;
    // packet ="R_data:";
    int i = 0;
    // while(i < rxSize)
    // {
      // packet += rxpacket[i];
      packet_t = temperature;
      packet_h = humidity;
      packet_r = rain;
      packet_w = windspeed;

      // packet += humidity;
      // i++;
    // }
    packSize = "R_Size: ";
    packSize += String(rxSize,DEC);
    packSize += " R_rssi: ";
    packSize += String(Rssi,DEC);
    // send_num = "send num: ";
    // send_num += String(txNumber,DEC);
    factory_display.drawString(0, 10, packet_t);
    factory_display.drawString(0, 20, packet_h);
    factory_display.drawString(0, 30, packet_r);
    factory_display.drawString(0, 40, packet_w);
    factory_display.drawString(0, 50, packSize);
    // factory_display.drawString(0, 50, send_num);
    factory_display.display();
    delay(10);
    factory_display.clear();

    if((rxNumber%2)==0)
    {
      digitalWrite(LED, HIGH);  
    }
    else if((rxNumber%2)!=0)
    {
      digitalWrite(LED, LOW);
    }
  }
  switch(state)       // Controle de estados do LoRa
    {
      // Serial Monitor Change States
      case STATE_TX:
        delay(1000);

        get_temp();
        get_umi();
        get_rain();
        get_nivel();

        if (millis() - lastSend > 5000){
          get_wind();

          // Serial.println("Sending packet !!!");

          // send_packets_received();
          delay(100);
          
          // Envio de pacotes através da rede LoraWan
          txNumber++;

          memcpy(buffer, &temperatura_lida, sizeof(float));
          memcpy(buffer + sizeof(float), &umidade_lida, sizeof(float));
          memcpy(buffer + sizeof(float) * 2, &volume_coletado, sizeof(float));
          memcpy(buffer + sizeof(float) * 3, &windspeed, sizeof(float));
          memcpy(buffer + sizeof(float) * 4, &metros, sizeof(float));
          memcpy(buffer + sizeof(float) * 5, &id, sizeof(float));

          // sprintf(txpacket,"hello %d,Rssi:%d",txNumber,Rssi);
          // Serial.printf("\r\nsending packet \"%s\" , length %d\r\n",buffer, strlen(buffer));
          Radio.Send( (uint8_t *)buffer, strlen( (char *)buffer) );

          // sendMessage();  // Envio de mensagem através da rede Mesh
          lastSend = millis();
        }

        state=LOWPOWER;
        break;
      case STATE_RX:
        Serial.println("into RX mode");
        Radio.Rx( 0 );
        state=LOWPOWER;
        break;
      case LOWPOWER:
        Radio.IrqProcess( );
        break;
      default:
        break;
    }
}
