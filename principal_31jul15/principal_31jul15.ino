  /* 
This code controls the watering system inside a greenhouse and surroundings. 
The watering system consists in:
(1) drip tubing (4 tubes, holes at 20 cm distance)
(2) sprinkler (6 outlets)
(3) valve to garden
(4) valve to soil plantation
(5) valve to water reservoirs
(6) valve to pump

To decide when turn the system on, some sensors are available:
(1) LDR (x2) - sun light
(2) DS18B20 (x2) - temperature (fast)
(3) DHT22 (x2) - temperature and humidity (slow)
(4) Moisture sensor (x4)
(5) Ultrasound sensor (x1) - water level at reservoir

General idea: environmental data are collected to decide when watering will be turned on!
This decision depends on the day/night cycle, soil moisture and temperature.
Before watering system be turned on, water content must be verified.
Sprinklers will be used to control the air temperature.

written by Leandro, André and Zacha
last revision: 22 jul 15
*/

//libraries
#include <DHT.h>            // DHT22
#include <OneWire.h>        // DS18B20
#include <DallasTemperature.h>  // DS18B20
#include <Ultrasonic.h>     // HC-SR04
#include <SD.h> 		// cartao SD
#include <SPI.h>	    // communication with SD card
#include <Wire.h>           // RTC

//#include <Dhcp.h>  // internet access
//#include <Dns.h>
//#include <Ethernet.h>       // internet access
//#include <EthernetClient.h>
//#include <EthernetServer.h>
//#include <EthernetUdp.h>
//#include <util.h>

// Port definitions
// to switch valve relays (relay board)
#define Vr1 24   // enable relay board (5V)
#define V110 25  // digital pin to transformer 110>24 volts
#define Vsp 26   // digital pin for sprinkler valve
#define Vdr 27   // digital pin for drip valve

//to switch pump's relays
#define Vr2 33  // enable relay board (5V)
#define Vpp 30  // enable pump

// to switch moisture sensor (MS) (YL38: Gnd + Vcc + data) (YL69: sensor - 2 wires)
// MSi = digital pin to switch MS´s VCC (the MS can not remain HIGH all time, to avoid corrosion)
#define MS1 46  // switchable Vcc MS1  //ports ok: 46, 47, 48, 49
#define MS2 47  // switchable Vcc MS2  //ports with problem: 50, 51, 52, 53 (I don´t know which problem!)
#define MS3 48  // switchable Vcc MS3
#define MS4 49  // switchable Vcc MS4

// analogic port - MS data (YL38 to Arduino)
#define MSd1 A8   // MS1 data
#define MSd2 A9   // MS2 data
#define MSd3 A10  // MS3 data
#define MSd4 A11  // MS4 data

// to switch LDRs (digital pin) and to receive data (analogic pin)
#define DLDR1 44  //digital pin to switch LDR1
#define DLDR2 45  //digital pin to switch LDR2
#define LDR1 A0  //LDR1 data
#define LDR2 A1  //LDR2 data

// to swtich/read DHT22s
#define DHT22_1 43    // DHT22_1 data (inside green house)
#define DHT22_2 41    // DHT22_1 data (inside green house)
#define DDHT 42       //switchable Vcc (both DHT22)
#define DHT_Type DHT22       // definition used in DHT´s library
DHT dht1(DHT22_1,DHT_Type);  // associating pointer dht1 to a DHT sensor
DHT dht2(DHT22_2,DHT_Type);  // associating pointer dht2 to a DHT sensor

// to switch/read Ultrasound sensor (US), SD and DHT
#define DUS 7           // VCC (continuous): some problems were observed when US was switched using a digital pin!
#define TRIGGER  6	// Ultrasound trigger
#define ECHO     5     	// Ultrasound echo
Ultrasonic ultrasonic(TRIGGER, ECHO);  // seting Ultrasound device

//#define CS 4	       	  //standard pin to Arduino, to use SD card
#define arquivo "data.txt" //file name, to be saved on SD card
#define arquivo2 "007.txt"  //L
File arq;     // pointer to file (saved on SD card)
File arq2; //L  Segundo arquivo SD - 007

#define DDHT3 8       // DHT3 Vcc (used together the Ultrasound sensor)
#define DHT22_3 9     // DHT22_1 data (air temperature, sound transmission)
#define DHTTYPE2 DHT22  // DHT22
DHT dht3(DHT22_3, DHTTYPE2);

// to switch RTC
#define enderecoI2C 0x68 // address "0x68" saved on enderecoI2C  (I2C bus)

// to switch/read DS18B20 (parasite mode)
#define DDS 3                  // switchable Vcc for both DS18B20
#define ONE_WIRE_BUS 2         // Data wire is plugged into pin 3 on the Arduino (defined by user)
#define TEMPERATURE_PRECISION_0 12  // resolution: 9 = 0.50 / 10 = 0.25 / 11 = 0.125 / 12 = 0.0625 
#define TEMPERATURE_PRECISION_1 12 
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature.
DeviceAddress DS0, DS1; // we are using only 2 devices (DS18B20)

// definition of variables  (used on measurements and statistics)
#define nread 50  // sample size to statistics (average value and standard deviation)
int i, j;       // integer counter (auxiliaries)
int ww;         //used whenever a MS-sensor is ignored
int wd;         //used whenever a DS-sensor is ignored


float H1[nread], H2[nread], T1[nread], T2[nread];  // auxiliary vectors to read DHTs
float TD1[nread], TD2[nread];   // auxiliary vectors to read DS18B20s
float L1[nread], L2[nread];   // auxiliary vectors to read LDRs
float M1[nread], M2[nread], M3[nread], M4[nread];  // auxiliary vectors to read Ms
float US[nread];  // auxiliary vectors to read US
float T3[nread];  // auxiliary vectors to read DHT3

float average, uncert;
float H1av, H1sd, H2av, H2sd, T1av, T1sd, T2av, T2sd; // averaged value and standard deviation - temperature and humidity - DHT22
float TD1av, TD1sd, TD2av, TD2sd;     // averaged value and standard deviation - temperature - DS18B20
float L1av, L1sd, L2av, L2sd;         // averaged value and standard deviation - sun light - LDR
float M1av, M1sd, M2av, M2sd, M3av, M3sd, M4av, M4sd;   // averaged value and standard deviation - Soil Moisture - YL38, YL69
float USav, USsd;  // averaged value and standard deviation - US
float T3av, T3sd;  // averaged value and standard deviation - temperature - DHT22 near Ultrasound

float sH = 1.0;  // intrinsic sensor precision: DHT - humidity
float sT = 0.5;  // intrinsic sensor precision: DHT - temperature
float sTD1, sTD2; //defined inside function startDS() = resolution/2
float sL = 2.0;   // intrinsic sensor precision: LDR
float sM = 5;    // intrinsic sensor precision: MS - soil moisture
float sU = 0.1;  // intrinsic sensor precision: US

// definition of variables  (used on decision steps)
float Lcv = 400.; // LDR critical value
float Mcv = 700.; // MS critical value
float Tcv = 35.; // DS critical value

// delay time in each step
int delaydripon  = 90000;  // drip system on (watering): 1.5 minutes
int delaydripoff = 600000;  // drip system off (diffusion): 10 minutes
float sptimeon  = 120.;   // (in SECONDS) sprinkler system on (watering): 2 minutes
float sptimeoff = 120.;   // (in SECONDS) sprinkler system off (estabilizing): 2 minutes

int Lw = 0;  // LDR state: 0 = night; 1 = day
int Mw = 0;  // MS state: 0 = wet ; 1 = dry
int Tw = 0;  // DS state: 0 = cold; 1 = warm

float avs;  // averaged value to make decision (turn on-off)
//z% acho melhor criar uma funcao para analisar a validade dos valores medidos!

// RTC
byte sec, mnt, hour, week, dmonth, month, year; // date and time

// Variables to time control
String hour1, mnt1, sec1;
byte day1, month1, year1;

// definition of functions
void startDS();  // to initialize DS18B20
void printAddress(DeviceAddress deviceAddress);  // to read and print DS18B20 addresses
float statistics(float X[], float sX);  // to calculate averaged values and standard deviations
float sprinkler();  // to turn on and off sprinkler system
float drip();   // to turn on anf off drip system
void configureModule();
void imprimeDadosModulo();
byte decToBcd(byte val); 
byte bcdToDec(byte val); 
String insertZero(byte dado);

/*
// ******************* Repassar com Aildo: como descobrir o MAC address e o definir o IP??? Porque 33000????
// Ethernet settings: to connect Arduino to Hub
// Enter a MAC address and IP address for your controller.
// The IP address will be dependent on your local network:
    byte mac[] = { 0x18, 0x03, 0x73, 0x70, 0x71, 0x11 };
    IPAddress ip(192, 168, 2, 200);
// Initialize the Ethernet server library with the IP address and port you want to use 
// (port 80 is default for HTTP):
 EthernetServer server(33000);
// ******************* Repassar com Aildo: como descobrir o MAC address e o definir o IP??? Porque 33000????
*/


void setup()
{
  Serial.begin(9600);  // opening serial communication
//  while (!Serial) {;} // wait for serial port to connect. Needed for Leonardo only
  
  // preparing pins to make available a level HIGH (turn on)
  pinMode(DLDR1,OUTPUT);  // Vcc LDR1
  pinMode(DLDR2,OUTPUT);  // Vcc LDR2
  pinMode(DDHT,OUTPUT);  // Vcc both DHT22
  pinMode(DDS,OUTPUT);   // Vcc both DS18B20
  pinMode(Vr1, OUTPUT);  // Vcc greenhouse's relays
  pinMode(Vsp,OUTPUT);   // relay sprinkler
  pinMode(Vdr,OUTPUT);   // relay drip
  pinMode(V110,OUTPUT);  // relay trafo 110V
  pinMode(Vr2, OUTPUT);  // Vcc pump's relays
  pinMode(Vpp, OUTPUT);  // Vcc pump
  pinMode(MS1,OUTPUT);   // Vcc moisture sensor 1
  pinMode(MS2,OUTPUT);   // Vcc moisture sensor 2
  pinMode(MS3,OUTPUT);   // Vcc moisture sensor 3
  pinMode(MS4,OUTPUT);   // Vcc moisture sensor 4
  pinMode(DDHT3, OUTPUT);  // Vcc DHT3 (close to US
  pinMode(DUS,OUTPUT);   // Vcc Ultrasound
  pinMode(ECHO, INPUT); // pin ECHO 
  pinMode(TRIGGER, OUTPUT); // pin TRIGGER
  pinMode(53, OUTPUT); //Arduino Mega requires pin 53 as output, whenever Ethernet Shield is in use!
 
  // initializing SD card
  if (!SD.begin(4)) {Serial.println("initialization failed!"); return;}
  Serial.println("SD initialization done.");
  
  dht1.begin(); dht2.begin(); dht3.begin();  // DHT22 initializing
  startDS();  // DS18B20 initialization
  Wire.begin(); //RTC DS1307 initialization
//  configureModule();  //to set RTC module to a specific date and time

/*  
// start the Ethernet connection and the server:
  Ethernet.begin(mac, ip);
  server.begin();
  Serial.print("The WWW server is at ");
  Serial.println(Ethernet.localIP());      
*/
  Serial.println("Enjoy your data and have a nice day!!"); 
  Serial.println(); 
}  // end void setup


// **************************************************************************************************************************
void loop()
{
  // Data reading (RTC DS1307)   
  imprimeDadosModulo();    //chamando a função para imprimir na tela os dados gravados no módulo
  
  arq2 = SD.open(arquivo2,FILE_WRITE);  //L
  if(arq2) {      
    Serial.println("007 on"); 
//    arq2.println("-----------------------------------------------------------------------------------------"); 
    arq2.print("Date: "); arq2.print(day1); arq2.print("/"); arq2.print(month1); arq2.print("/"); arq2.print(year1); arq2.print(" - ");
    arq2.print(hour1); arq2.print(":"); arq2.print(mnt1); arq2.print(":"); arq2.println(sec1);
  
              
    // reading US
    digitalWrite(DUS, HIGH); delay(5000); //turning US on
    for(i=0; i<nread; i++)   {US[i] = (ultrasonic.Ranging(CM)); // Serial.print(US[i],1); Serial.print(" ");	// reading Ultrasound } 
       delay(500);}
    digitalWrite(DUS, LOW); // turning US off
     
    Serial.print("US: ");statistics(US,sU); USav = average; USsd = uncert;
    Serial.print(average,1); Serial.print(" +- "); Serial.println(uncert,1);
    
    // Reading DHT22 3
    digitalWrite(DDHT3,HIGH); delay(500); //it is important a delay > 450ms
    for(i=0; i<nread; i++) {T3[i] = dht3.readTemperature();} //reading humidity and temperature on DHT22_1
    digitalWrite(DDHT3,LOW);  // turninf off DHT sensors
   
    Serial.print("T3: "); statistics(T3,sT); T3av = average; T3sd = uncert;
    Serial.print(average,1); Serial.print(" +- "); Serial.println(uncert,1);  Serial.println(" "); 
  
                         
// Data reading (2xDHT,2xDS18B20, 2xLDR, 4xMS)

    // reading LDRs
    digitalWrite(DLDR1,HIGH); digitalWrite(DLDR2,HIGH);   // turning LDRs on   
    for(i=0; i<nread; i++)   { L1[i] = analogRead(LDR1); L2[i] = analogRead(LDR2); } 
    delay(3000);
    digitalWrite(DLDR1,LOW); digitalWrite(DLDR2,LOW);   // turning LDRs off   

    statistics(L1,sL); L1av = average; L1sd = uncert; 
    statistics(L2,sL); L2av = average; L2sd = uncert; 
  // end reading LDRs

// day ligth?
// Only L2 (upper sensor) is considered to make decision. Maybe its standard deviation can be used to detect problems in sensor!
//    avs = (L1av + L2av)/2.; if (avs > Lcv) Lw = 1;  // 0 = night; 1 = day
    if (L2av > Lcv) Lw = 1;  // 0 = night; 1 = day
    Serial.print("LDR: "); Serial.print(L2av,0); Serial.print("\t"); 
    if (Lw == 0) {Serial.print("night");} else {Serial.print(" day ");} Serial.print("\t"); 
    if (Mw == 0) {Serial.print(" wet ");} else {Serial.print(" dry ");} Serial.print("\t"); 
    if (Tw == 0) {Serial.println(" cold");} else {Serial.println(" warm");} 

// reading MS´s
    digitalWrite(MS1,HIGH); digitalWrite(MS2,HIGH); digitalWrite(MS3,HIGH); digitalWrite(MS4,HIGH); // turning Moisture Sensors (MS) on      
    delay(1000); // 1 second delay - to establish electrical measurements
    for(i=0; i<nread; i++) {M1[i] = analogRead(MSd1); M2[i] = analogRead(MSd2); M3[i] = analogRead(MSd3); M4[i] = analogRead(MSd4);} delay(5000);
    digitalWrite(MS1,LOW); digitalWrite(MS2,LOW); digitalWrite(MS3,LOW); digitalWrite(MS4,LOW); // turning Moisture Sensors (MS) off
    
    statistics(M1,sM); M1av = average; M1sd = uncert; 
    statistics(M2,sM); M2av = average; M2sd = uncert; 
    statistics(M3,sM); M3av = average; M3sd = uncert; 
    statistics(M4,sM); M4av = average; M4sd = uncert; 
    // end reading MSs
  
  //Soil moisture
   // All MS-sensor > 900 and < 100 (arbitrary values) will be ignored.
   ww = 0;
   if ((M1av>900) || (M1av<100)) {arq2.print("Error M1 "); ww++; M1av=0;}
   if ((M2av>900) || (M2av<100)) {arq2.print("Error M2 "); ww++; M2av=0;}
   if ((M3av>900) || (M3av<100)) {arq2.print("Error M3 "); ww++; M3av=0;}
   if ((M4av>900) || (M4av<100)) {arq2.print("Error M4 "); ww++; M4av=0;}
   
   if (ww<4) {
     avs = (M1av + M2av + M3av + M4av)/(4-ww);
     if (avs > Mcv) Mw = 1; // 0 = wet ; 1 = dry
     Serial.print("MS : "); Serial.print(avs,0); Serial.print("\t"); 
     if (Lw == 0) {Serial.print("night");} else {Serial.print(" day ");} Serial.print("\t"); 
     if (Mw == 0) {Serial.print(" wet ");} else {Serial.print(" dry ");} Serial.print("\t"); 
     if (Tw == 0) {Serial.println(" cold");} else {Serial.println(" warm");} 

     if (Lw == 1 && Mw == 1) {Serial.println("... "); drip();}  // drip system
   }
   else {Serial.print ("MS-sensors in problem!!"); arq2.print("All MS-sensors are in problem!!");}

/* DHT sensors were eliminated 
   // Reading DHT22
   digitalWrite(DDHT,HIGH); delay(500); //it is important a delay > 450ms
   for(i=0; i<nread; i++)
   {
     H1[i] = dht1.readHumidity(); T1[i] = dht1.readTemperature(); //reading humidity and temperature on DHT22_1
     H2[i] = dht2.readHumidity(); T2[i] = dht2.readTemperature(); //reading humidity and temperature on DHT22_2
   }
   digitalWrite(DDHT,LOW);  // turninf off DHT sensors
  
   statistics(H1,sH); H1av = average; H1sd = uncert;  
   statistics(H2,sH); H2av = average; H2sd = uncert;  
   statistics(T1,sT); T1av = average; T1sd = uncert;  
   statistics(T2,sT); T2av = average; T2sd = uncert;  
   // end reading DHT
*/


   // reading DS18B20   
   digitalWrite(DDS,HIGH); delay(500);  // turning DS on 
   wd = 0;
   avs = 0.;
   // Error in DS can be identified asking it about its address
   sensors.requestTemperatures(); // call sensors.requestTemperatures() to issue a global temperature request to all devices on the bus  
   if (!sensors.getAddress(DS0, 0)) {arq2.print("Error DS0 "); TD1av = 0; TD1sd = 0; wd++; }
   else{ 
     for(i=0; i<nread; i++) {TD1[i] = sensors.getTempC(DS0);} //reading temperature on DS18B20s
     statistics(TD1,sTD1); TD1av = average; TD1sd = uncert; avs += TD1av;
   }
  
   if (!sensors.getAddress(DS1, 0)) {arq2.print("Error DS1 "); TD2av = 0; TD2sd = 0; wd++;}
   else{ 
     for(i=0; i<nread; i++) {TD2[i] = sensors.getTempC(DS1);} //reading temperature on DS18B20s
     statistics(TD2,sTD2); TD2av = average; TD2sd = uncert; avs += TD2av;
   }
   digitalWrite(DDS,LOW);   // turning DS18B20 off

   //Air Temperature
   if (wd == 2) {Serial.print("both DS in trouble!!");}
   else{ 
     if ((wd == 0) && (abs(TD1av-TD2av)>=2)) {arq2.print("delta_T > 2: "); arq2.print(TD1av); arq2.print(TD2av);}  //if Delta_T > 2, DS in problem
     avs /= (2-wd);
   }

     if (avs > Tcv) Tw = 1; // 0 = cold ; 1 = warm
     Serial.print("T  : "); Serial.print(avs,1); Serial.print("\t"); 
     if (Lw == 0) {Serial.print("night");} else {Serial.print(" day ");} Serial.print("\t"); 
     if (Mw == 0) {Serial.print(" wet ");} else {Serial.print(" dry ");} Serial.print("\t"); 
     if (Tw == 0) {Serial.println(" cold");} else {Serial.println(" warm");} 
     if (Lw == 1 && Tw == 1) {Serial.println("... "); sprinkler();} // sprinkler system
   // end reading DS18B20


/*
   // estabelecendo contato com a internet
   EthernetClient client = server.available();  //Verifica se tem dados disponíveis no servidor (Arduino)
   if (client) {  //verifica se o cliente (browser) se conectou. Se sim, executa o if!
   // Uma solicitação HTTP do cliente (browser) terminará com um linha em branco, tendo ao seu final um caracter
   // de nova linha. A variável BlankLine será usada para determinar se atingiu ou não o fim dos dados.
     boolean BlankLine = true; 
   
     while (client.connected()) {
       if (client.available()) {
         char c = client.read();
           // if you've gotten to the end of the line (received a newline
           // character) and the line is blank, the http request has ended,
           // so you can send a reply
           if (c == '\n' && BlankLine) {
             // send a standard http response header
             client.println("HTTP/1.1 200 OK");
             client.println("Content-Type: text/html");
             client.println("Connection: close");  // the connection will be closed after completion of the response
             client.println("ReMSdesh: 5");  // reMSdesh the page automatically every 5 sec
             client.println();
             client.println("<!DOCTYPE HTML>");
             client.println("<html>");
             client.print("\t");
             client.print(H1av); client.print(" "); client.print(H1sd); client.print(" ");
             client.print(H2av); client.print(" "); client.print(H2sd); client.print(" ");
             client.print(T1av); client.print(" "); client.print(T1sd); client.print(" ");
             client.print(T2av); client.print(" "); client.print(T2sd); client.print(" ");
             client.print(TD1av); client.print(" "); client.print(TD1sd); client.print(" ");
             client.print(TD2av); client.print(" "); client.print(TD2sd); client.print(" ");
             client.print(L1av); client.print(" "); client.print(L1sd); client.print(" ");
             client.print(L2av); client.print(" "); client.print(L2sd); client.print(" ");
             client.print(M1av); client.print(" "); client.print(M1sd); client.print(" ");
             client.print(M2av); client.print(" "); client.print(M2sd); client.print(" ");
             client.print(M3av); client.print(" "); client.print(M3sd); client.print(" ");
             client.print(M4av); client.print(" "); client.print(M4sd); 
             client.println(" ");
         }
         if (c == '\n') {BlankLine = true;}   // you're starting a new line
         else if (c != '\r') {BlankLine = false;} // you've gotten a character on the current line
       } // if client.available
       } // while
       
       delay(1); // give the web browser time to receive the data
       client.stop();   // close the connection:
       //Serial.println("client disonnected");
       } //end if(client)
 
*/

          arq2.close();  //L
          Serial.println("007 off");Serial.println(" ");
        } 
        else {Serial.println("Error 007");}

//L  Escrever o cabecalho do arq no void setup; escrever os valores utilizando /t para facilitar a sua leitura no excel

//        arq = SD.open("res.txt",FILE_WRITE);
        arq = SD.open(arquivo,FILE_WRITE);
        if(arq) {      
          Serial.println("Escrevendo no cartao..."); 
          arq.println("-----------------------------------------------------------------------------------------"); 
          arq.print("Date: "); arq.print(day1); arq.print("/"); arq.print(month1); arq.print("/"); arq.print(year1); arq.print("  ");
          arq.print(hour1); arq.print(":"); arq.print(mnt1); arq.print(":"); arq.println(sec1); arq.println(" ");
          
          arq.print("US: "); arq.print(USav); arq.print(" +- "); arq.print(USsd, 1); arq.print("\t"); arq.print("T3: "); arq.print(T3av); arq.print(" +- "); arq.println(T3sd, 1);
          arq.print("L2: "); arq.print(L2av); arq.print(" +- "); arq.println(L1sd, 1);
          arq.print("TD1: "); arq.print(TD1av,1); arq.print(" +- "); arq.println(TD1sd, 1);
          arq.print("TD2: "); arq.print(TD2av,1); arq.print(" +- "); arq.println(TD2sd, 1);
 //         arq.print("H1: "); arq.print(H1av); arq.print(" +- "); arq.println(H1sd, 1); 
 //         arq.print("H2: "); arq.print(H2av); arq.print(" +- "); arq.println(H2sd, 1);
          arq.print("M1: "); arq.print(M1av); arq.print(" +- "); arq.println(M1sd, 1);
          arq.print("M2: "); arq.print(M2av); arq.print(" +- "); arq.println(M2sd, 1);
          arq.print("M3: "); arq.print(M3av); arq.print(" +- "); arq.println(M3sd, 1);
          arq.print("M4: "); arq.print(M4av); arq.print(" +- "); arq.println(M4sd, 1);
          arq.println(" ");        
       
         if (Lw == 1 && Mw == 1) {arq.println("Drip foi ativado ");} 
         if (Lw == 1 && Tw == 1) {arq.println("Sprinkler foi ativado ");} // sprinkler system
       
          arq.close();
          Serial.println(" Feito");Serial.println(" ");
        } 
        else {Serial.println("Nao foi possivel abrir o txt");}
         
 Lw = Mw = Tw = 0; 
 
 delay(1800000);
 
}  // end loop

//***************** functions *******************************************************************************

// function to initialize DS18B20  *****************************************************************
void startDS()
{
  Serial.println("Hello World!! Lets start counting and identifying all devices connected _through_ OneWire!"); 
  Serial.println("");

  digitalWrite(DDS,HIGH);  // turning on both DS18B20
  delay(500);  // 0.5 seconds delay to estabilize DS18B20 (I dont know if really necessary!!)

  sensors.begin();	// start up the library (? maybe a library defined in DallasTemperature!)

  // Associating resolution for each sensor (defined before)
  sensors.setResolution(DS0, TEMPERATURE_PRECISION_0);
  sensors.setResolution(DS1, TEMPERATURE_PRECISION_1);

  // counting the nunber os sensors in the bus
  Serial.print("I have detected "); 
  Serial.print(sensors.getDeviceCount(), DEC); 
  Serial.println(" devices in your system!");

  // report parasite power requirements
  Serial.print("Parasite power is: "); 
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

  // identifying and printing address for each device
  // sensors.getAddress(DSx, x) == 1 means: device address found
  if (!sensors.getAddress(DS0, 0)) {Serial.println("Unable to find address for device 0");}
  else 
  {
    Serial.print("Device 0 address: "); printAddress(DS0); Serial.println();
    Serial.print("Device 0 resolution: "); Serial.print(sensors.getResolution(DS0), DEC); Serial.println();
  }

  if (!sensors.getAddress(DS1, 1)) {Serial.println("Unable to find address for device 1");}
  else 
  {
    Serial.print("Device 1 address: "); printAddress(DS1); Serial.println();
    Serial.print("Device 1 resolution: "); Serial.print(sensors.getResolution(DS1), DEC); Serial.println();
  }
    
    sTD1 = (sensors.getResolution(DS0), DEC)/2.;  // intrinsic sensor precision: DS18B20_1 - temperature
    sTD2 = (sensors.getResolution(DS1), DEC)/2.;  // intrinsic sensor precision: DS18B20_2 - temperature
    
    digitalWrite(DDS,LOW);  // turning off both DS18B20
}  // end of startDS()


// function to print a device address *****************************************************************
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

float statistics(float X[], float sX)
{
  average = uncert = 0.;
  for(i=0; i<nread; i++) {average = average + X[i];}  // adding all measurements
  average = average/nread;  // averaged values
  
  for(i=0; i<nread; i++) {uncert = uncert + (average - X[i])*(average - X[i]);}  // adding square deviations (variance)  
  uncert = uncert/(nread -1);  // variance
  uncert = sqrt(uncert + sX*sX);  // adding sensor precision
        
 Serial.print(average,1); Serial.print(" +- "); Serial.println(uncert,1);
}


//z? implementar delays como parametros
float drip()  
{  
  Serial.println("Dropping time! ");
  digitalWrite(Vr2, HIGH); delay(500);
  digitalWrite(Vpp, HIGH);  // pump relay
  digitalWrite(Vr1, HIGH);  // board relay
  digitalWrite(V110, HIGH);  // 110 to 24V relay
  delay(5000); //10 seconds to stabilize pump and relays
  digitalWrite(Vdr, HIGH);  // opening drip valve
  delay(delaydripon); // drip time
  
  digitalWrite(Vpp, LOW);
  digitalWrite(Vr2, LOW);
  digitalWrite(Vdr, LOW);
  digitalWrite(Vr1, LOW);
  digitalWrite(V110, LOW);
  delay(delaydripoff); // time to water difusion throughout soil
  Serial.println("End drip");
}

float sprinkler() // function to enable sprinkler
{
  float time0, timenow, delta;  // time data and interval, with sprinker turned on
  float t1, t2, tav; // auxiliars
  
  Serial.println("sprinkler time! ");
  digitalWrite(Vr2, HIGH); delay(500);
  digitalWrite(Vpp, HIGH);  // pump relay
  digitalWrite(Vr1, HIGH);  // board relay
  digitalWrite(V110, HIGH);  // 110 to 24V relay
  delay(5000); //10 seconds to stabilize pump and relays
  digitalWrite(DDS,HIGH); delay(500);  // turning DS on (to follow temperature changes due sprinkler watering)   
  sensors.requestTemperatures(); // call sensors.requestTemperatures() to issue a global temperature request to all devices on the bus
  
  digitalWrite(Vsp, HIGH);  // opening sprinkler valve
  unsigned long time = millis();  // current time
  time0 = time/1000.;  //initial time, in seconds
  delta = 0.;
  while(delta < sptimeon) {
    t1 = sensors.getTempC(DS0); //reading temperature on DS18B20_1
    t2 = sensors.getTempC(DS1); //reading temperature on DS18B20_2
    tav = (t1+t2)/2.;
    Serial.print(delta); Serial.print(" "); Serial.println(tav);
    delay(900);  // to force around 1 second for each temperature measurement
    timenow = millis()/1000.;
    delta = timenow - time0;
  } // end while
  
  // turning off sprinkler system
  digitalWrite(Vsp, LOW);
  digitalWrite(V110, LOW);
  digitalWrite(Vr1, LOW);
  digitalWrite(Vpp, LOW);
  digitalWrite(Vr2, LOW);
  Serial.println("End parcial"); 

  // temperarute measurement still remains!  
//  sptimeon += sptimeoff;
  while(delta < (sptimeon + sptimeoff)) {
    t1 = sensors.getTempC(DS0); //reading temperature on DS18B20_1
    t2 = sensors.getTempC(DS1); //reading temperature on DS18B20_2
    tav = (t1+t2)/2.;
    Serial.print(delta); Serial.print(" "); Serial.println(tav);
    delay(900);  // to force around 1 second for each temperature measurement
    timenow = millis()/1000.;
    delta = timenow - time0;
  } // end while
    
  digitalWrite(DDS,LOW);   // turning DS18B20 off
  Serial.println("End total"); 
}

// RTC module 
void configureModule()  // Date and time adjustment. 
{
  Wire.beginTransmission(enderecoI2C); //opening I2C address in recording mode
  Wire.write((byte)0x00);   //redefining pointer to the first register (0x00)
  
  // setting and writing data. For data less than 10, only one digit must be typed (ex: 9 hour = 9, not 09)
  // To write data in the RTC module, we should convert data from decimal to binary
  sec = 0;  Wire.write(decToBcd(sec));      //converting seconds
  mnt = 43; Wire.write(decToBcd(mnt));      //converting minutes.
  hour = 7; Wire.write(decToBcd(hour));     //converting hours.
  week = 4; Wire.write(decToBcd(week));     //converting week day: sunday == "0"
  dmonth = 23; Wire.write(decToBcd(dmonth)); //converting day
  month = 7; Wire.write(decToBcd(month));   //converting month
  year = 15; Wire.write(decToBcd(year));    //converting year
  Wire.endTransmission();                   //closing recording mode
}

void imprimeDadosModulo() // to read date and time recorded at RTC module, and print them in the Serial Monitor.
{
  String adjustSec;  // these strings will be used to write date and time with 2 digits:
  String adjustMin;  // ex: 9:58:5 -> 09:58:05
  String adjustHour;
  String adjustDay;
  String adjustMonth;
  
  // Reading data at RTC module:
  Wire.beginTransmission(enderecoI2C); //opening communication I2C in recording mode
  Wire.write((byte)0x00); //redefining pointer to the first register (0x00)
  Wire.endTransmission(); //closing recording mode
  Wire.requestFrom(enderecoI2C, 7);   //opening communication I2C in reading mode to read the 7-byte-data

  //reading data from RTC module after conversion binary to decimal
  sec = bcdToDec(Wire.read() & 0x7f);   //reading converted seconds. Some data need masks because some bits are control-bits
  mnt = bcdToDec(Wire.read());          //reading converted minutes 
  hour = bcdToDec(Wire.read() & 0x3f);  //reading converted hour. Some data need masks because some bits are control-bits. This mask is to apply the 24h mode.
  week = bcdToDec(Wire.read());         //reading converted weekday: sunday == "0".
  dmonth = bcdToDec(Wire.read());       //reading converted day
  month = bcdToDec(Wire.read());        //reading converted month
  year = bcdToDec(Wire.read());         //reading converted year
 
  Serial.println("-------------------------------------------------------------------------------------------");
  Serial.print("Current time: ");
  adjustHour += insertZero(hour); hour1 = adjustHour;
  Serial.print(adjustHour); Serial.print(":");
  adjustMin += insertZero(mnt); mnt1 = adjustMin;
  Serial.print(adjustMin); Serial.print(":");
  adjustSec += insertZero(sec); sec1 = adjustSec;
  Serial.println(adjustSec);
  
  Serial.print("Day of the week: ");
  switch(week)
  {
    case 0: Serial.println("Sunday"); break; 
    case 1: Serial.println("Monday"); break;
    case 2: Serial.println("Tuesday"); break;
    case 3: Serial.println("Wednesday"); break;
    case 4: Serial.println("Thursday"); break; 
    case 5: Serial.println("Friday"); break;   
    case 6: Serial.println("Saturday"); break; 
  }
  
  Serial.print("Current date: ");
  adjustDay += insertZero(dmonth); day1 = dmonth;
  Serial.print(adjustDay); Serial.print("/");
  adjustMonth += insertZero(month); month1 = month;
  Serial.print(adjustMonth); Serial.print("/");
  Serial.println(year); year1 = year;
  Serial.println();
}

byte decToBcd(byte val)  // to convert decimal to binary data
{ return ( (val/10*16) + (val%10) ); }
 
byte bcdToDec(byte val)  // to convert binary to decimal data
{  return ( (val/16*10) + (val%16) ); }


String insertZero(byte dado)  //to insert a "0" for data with one digit only
{
  String dadoAjustado;
  if (dado < 10) {dadoAjustado += "0";}  //concatenate a "0" to dado
  dadoAjustado += dado;   // formatting the new value
  return dadoAjustado; 
}



