//Estação meteorológica com integração IoT para auxiliar no manejo de irrigação

//Autor: Pedro Josefino Custódio de Araújo.
//-----------------------------------------------------------------------------------

//IoT
#include <WiFi.h>
#include <Wire.h>
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient/releases/tag/v2.3
#include <ArduinoJson.h> // https://github.com/bblanchon/ArduinoJson/releases/tag/v5.0.7
#include "Esp32MQTTClient.h"

//Conexão com o IBM Cloud
#define ORG "pmnxpr" // ID da organização 
#define DEVICE_TYPE "ESP32" // Tipo de dispositivo
#define DEVICE_ID "001" // ID do dispositivo
#define TOKEN "a-pmnxpr-z2jem5omlb"// Token de autenticação

//Comunicação IoT
char server[] = ORG ".messaging.internetofthings.ibmcloud.com";
char authMethod[] = "use-token-auth";
char token[] = TOKEN;
char clientId[] = "d:" ORG ":" DEVICE_TYPE ":" DEVICE_ID;
const char eventTopic[] = "iot-2/evt/status/fmt/json";
const char cmdTopic[] = "iot-2/cmd/led/fmt/json";


//DHT11
#include "DHT.h"
#define DHTPIN 15
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
float tempMedia = 0;
float tempMax = 0;
float tempMin = 0;
float umidMedia = 0;

//GYML8511
int UVOUT = 33;
int UV_3V3 = 35;
float uvIntDia = 0;

//BMP180
#include <Adafruit_BMP085.h>
Adafruit_BMP085 bmp;

//Sensor de umidade do solo
#define pinSensorA 32


#include <math.h>
int numLeituras=0;
int diadociclo=1; //Considerando que o sistema de irrigação será implementado no início do ciclo
int J=0;
float delta1;
float gama1; 
float es1;
float ea1; 
float latitude = -12.1483; //Latitude da cidade de Barreiras-BA
float rn1;
float EToPMF;
float Kc;
float ETc;
float ETca = 0;


//----------------------------------------------------------


#include <NTPClient.h> //Biblioteca NTPClient modificada
#include <WiFiUdp.h> //Socket UDP
 
//Fuso Horário
int timeZone = -3;

//Variáveis de controle
int hora_atual = 0;
int hora_ant = -1;
 
//Struct com os dados do dia e hora
struct Date{
    int dayOfWeek;
    int day;
    int month;
    int year;
    int hours;
    int minutes;
    int seconds;
};
 
//Socket UDP que a biblioteca usa para obter os dados do horário
WiFiUDP udp;
 
//Objeto que recupera os dados sobre o horário
NTPClient ntpClient(
    udp,                    //Socket UDP
    "0.br.pool.ntp.org",    //URL do server NTP
    timeZone*3600,          //Deslocamento do horário em relacão ao GMT 0
    60000);                 //Intervalo entre verificações online

//--------------------------------------------------------------------------------

WiFiClient wifiClient;
void callback(char* topic, byte* payload, unsigned int payloadLength)
{
  Serial.print("Messagem recebida [");
  Serial.print(topic);
  Serial.print("] ");
}
PubSubClient client(server, 1883, callback, wifiClient);

void setup()
{
    Serial.begin(115200);

    //DHT11
    dht.begin(); //Inicializar o sensor de temperatura e umidade
  
    //GYML8511
   pinMode(UVOUT, INPUT);
   pinMode(UV_3V3, INPUT);

   //BMP180
   if (!bmp.begin()) {
   Serial.println("Erro: Sensor BMP180 não reconhecido.");
    while (1) {}
   }
   
   //---------------------------------------------------------------------------
    client.setCallback(callback);
    connectWiFi();
    setupNTP();
    mqttConnect();
 
    //Cria uma nova tarefa no core 0
    xTaskCreatePinnedToCore(
        wifiConnectionTask,     //Função a ser executada
        "wifiConnectionTask",   //Nome da tarefa
        10000,                  //Tamanho da memória disponível (em WORDs)
        NULL,                   //Parâmetros
        2,                      //Prioridade
        NULL,                   //Referência
        0);                     //Número do core
}

void setupNTP()
{
    //Inicializa o cliente NTP
    ntpClient.begin();
     
    //Espera pelo primeiro update online
    Serial.println("Aguardando pela primeira atualização.");
    while(!ntpClient.update())
    {
        Serial.print(".");
        ntpClient.forceUpdate();
        delay(500);
    }
 
    Serial.println();
    Serial.println("Primeira atualização completa.");
}

Date getDate()
{
    //Obtém os dados de data e hora por meio do cliente NTP
    char* strDate = (char*)ntpClient.getFormattedDate().c_str();
 
    //Passa os dados da string para a struct
    Date date;
    sscanf(strDate, "%d-%d-%dT%d:%d:%dZ", 
                    &date.year, 
                    &date.month, 
                    &date.day, 
                    &date.hours, 
                    &date.minutes,
                    &date.seconds);
    return date;
}

//Função que realiza a conexão WiFi
void connectWiFi()
{
    Serial.println("Conectando...");
 
    //Aqui a conexão WiFi é realizada com o SSID e senha da rede conectada, respectivamente
    WiFi.begin("PedroWIFI", "123456789");
     
    //Espera enquanto não estiver conectado
    while(WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(500);
    }
 
    Serial.println();
    Serial.print("Conectado a ");
    Serial.println(WiFi.SSID());
}

//Tarefa que realiza a verificação da conexão. Caso tenha perdido o contato, tenta reconectar
void wifiConnectionTask(void* param)
{
    while(true)
    {
        //Se a WiFi não está conectada
        if(WiFi.status() != WL_CONNECTED)
        {
            //Manda conectar
            connectWiFi();
        }
 
        //Delay de 100 ticks
        vTaskDelay(100);
    }
}

void mqttConnect()
{ // Funções conectar ao servidor
  if (!!!client.connected())
  { // Se não houver conexão com o servidor
    Serial.print("Reconectando ao servidor:");
    Serial.println(server);      // Indica o endereço do servidor

    while (!!!client.connect(clientId, authMethod, token) )
    {
      Serial.print(".");
      delay(500);
    }
    if (client.subscribe(cmdTopic))
    { // Se conseguir se Conectar ao cmdTopic
      Serial.println("OK");   // Escreve OK no monitor serial
    }
    else
    {
      Serial.println("Erro"); // Escreve erro no monitor serial
    }
  }
}

//Função que realiza uma média entre um determinado número de leituras de um sensor analógico
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 16;
  unsigned int runningValue = 0; 

  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return(runningValue);
}

//Função de mapeamento de um intervalo númerico para outro, com a utilização de variáveis do tipo 'float'
float mapfloat(float valor, float min_atual, float max_atual, float novo_min, float novo_max)
{
  return (valor - min_atual) * (novo_max - novo_min) / (max_atual - min_atual) + novo_min;
}


//----------------------------------------------------------------------------------------------------------------
int calcularDiaDoAno(int dia, int mes, int ano) {
  
  int diaMes[] = {31,28,31,30,31,30,31,31,30,31,30,31};

  //Testando se o ano é bissexto
  if (ano%4  == 0) {
    if (ano%100 != 0) {
      diaMes[1] = 29; //fevereiro
    }
    else { if (ano%400 == 0) {
        diaMes[1] = 29;
      }
    }
   }

  if(dia<1) return 0; //É retornado 0, caso haja erro
  
  int diaAno = 0;
  for (int i = 1; i < mes; i++) {
    diaAno += diaMes[i];
  }
  
  diaAno += dia;
  return diaAno;
}

float calcDelta(float T){
    float delta = 4098 * ( (0.6108 * exp((17.27 * T)/(T + 237.3)) ) / ( pow((T + 237.3), 2)));
    return delta;
}

float calcGama(float P){
    float gama = 0.665 * 0.001 * P;
    return gama;
}

float calcEs(float T){
    float es = 0.6108 * exp( ( (17.27 * T) / (T + 237.3) )  );
    return es;
}

float calcEa(float UR, float es){
    float ea = (es * UR) / 100;
    return ea;
}

float calcRn(float albedo, float uvInc, float Tmax, float Tmin, float ea, float z, float J, float lat){
  
  lat = (lat*PI)/180; //Convertendo graus para radianos
  
  float decSolar = 0.409 * sin((((TWO_PI/365) * J) - 1.39));

  float X = 1 - (pow(tan(lat), 2) * pow(tan(decSolar), 2));
  if(X<=0) X = 0.00001;

  float omegaS = (PI/2) - atan( (-tan(lat)*tan(decSolar)) / (sqrt(X)) );
  
  float dr = 1 + 0.033 * cos(((TWO_PI/365) * J));
  
  float ra = (118.08/PI) * dr * ((omegaS * sin(lat) * sin(decSolar)) + (cos(lat) * cos(decSolar) * sin(omegaS)));
  
  float rso = (0.75 + (2 * pow(10,(-5)) * z)) * ra;

  float rnl = (4.903 * pow(10,(-9))) * ((pow((Tmax + 273.16), 4) + pow((Tmin + 273.16), 4)) / 2 ) * (0.34 - (0.14 * sqrt(ea))) * ((1.35 * (uvInc/rso)) - 0.35);  

  float rns = (1-albedo) * uvInc;
  
  float rn = rns - rnl;

  return rn;
}

float calcETo(float delta, float rn, float gama, float es, float ea, float T, float u2){
  
    float ETo = ((0.408 * delta * rn) + (gama * (900/(T + 273)) * u2 * (es - ea))) / (delta + (gama*(1 + (0.34 * u2))));

    return ETo;
}

float kcMilho(int cicloTotal, bool climaSeco, int diaCiclo){
    //Foi utilizado a título de estudo a cultura do milho como base
    //Cada variável "faseX" corresponde à quantidade de dias de cada fase do ciclo
    int fase1 = 0.17 * cicloTotal; 
    int fase2 = 0.28 * cicloTotal;
    int fase3 = 0.33 * cicloTotal;
    int fase4 = 0.22 * cicloTotal;
    if(diaCiclo<0) return 0;
    else{
      if(climaSeco){
       if(diaCiclo<=fase1){
          return 1;
        }else if(diaCiclo<=(fase1+fase2)){
          return (1 + ((0.2 / fase2) * (diaCiclo - fase1)));
        }else if(diaCiclo<=(fase1+fase2+fase3)){
          return 1.2;
        }else if(diaCiclo<=(fase1+fase2+fase3+fase4)){
          return (1.2 - ((0.8 / fase4) * (diaCiclo - (fase1+fase2+fase3))));
        }else{
          return 0;
        }
     }else{
        if(diaCiclo<=fase1){
          return 0.4;
        }else if(diaCiclo<=(fase1+fase2)){
          return (0.4 + ((0.6 / fase2) * (diaCiclo - fase1)));
        }else if(diaCiclo<=(fase1+fase2+fase3)){
          return 1;
        }else if(diaCiclo<=(fase1+fase2+fase3+fase4)){
          return (1 - ((0.6 / fase4) * (diaCiclo - (fase1+fase2+fase3))));
        }else{
          return 0;
        }
     }
   }
}

void loop(){ 

    if (!client.loop())
    { // Se houver desconexão ao servidor
       mqttConnect();                     // Executa novamente a função para conectar ao servidor
    }
    
    //Recupera os dados sobre a data e horário
    Date date = getDate();
    hora_atual = date.hours;
    
    if(hora_ant != hora_atual){
       
       int uvLvl = averageAnalogRead(UVOUT);
       int Lvl3v3 = averageAnalogRead(UV_3V3);
  
        //Tomando como base a voltagem de 3.3V
        float voltSaida = (3.3 / Lvl3v3) * uvLvl;
  
        float uvInt = mapfloat(voltSaida, 0.99, 2.8, 0.0, 15.0); //Converte a voltagem para o nível de intensidade Ultra Violeta (UV)
         uvInt += 1.07; //Calibração
         if(uvInt<0) uvInt=0;

        float uvInt2 = uvInt; //uvInt2 é uma cópia de uvInt, antes das conversões

        //Conversões
        //mW/cm² para W/m²
        //uvInt*=10;
        //W/m² para J/(hora.m²)
        //uvInt*=3600;
        //J/(hora.m²) para MJ/(hora.m²)
        //uvInt/=1000000;  
        
        uvInt*=0.036; //Conversão direta       

        uvIntDia+=uvInt;
       //---------------------------------------------------------------------------------------------------------------

        float umidade = dht.readHumidity();
        
        float temperatura = dht.readTemperature();

        if(isnan(umidade) || isnan(temperatura)){
          Serial.println(F("Falha na leitura do sensor DHT"));
          
          return;
        }
        
        if(numLeituras==0){
          tempMax = temperatura;
          tempMin = temperatura;
        }  
        if(temperatura>tempMax)tempMax=temperatura;
        if(temperatura<tempMin)tempMin=temperatura;
        
        tempMedia+=temperatura;
        umidMedia+=umidade;

       //---------------------------------------------------------------------------------------------------------------

        float temperatura2 = bmp.readTemperature();

        float pressao = (bmp.readPressure())/1000; //Converte de Pa para KPa

        float altitude = bmp.readAltitude();

        //---------------------------------------------------------------------------------------------------------------

         //Sensor de umidade do solo
        float umidsolo = analogRead(pinSensorA)-1900; //A subtração de '1900' foi utilizada para a calibração de um valor mínimo próximo de zero

        //---------------------------------------------------------------------------------------------------------------
        hora_ant = hora_atual;

        Serial.println(WiFi.SSID().c_str());
        Serial.print(date.day); 
        Serial.print(F("/"));
        Serial.print(date.month);
        Serial.print(F("/"));
        Serial.print(date.year);
        Serial.print(F(" - "));
        Serial.print(date.hours);
        Serial.print(F(":"));
        Serial.print(date.minutes);
        Serial.println();

        

        Serial.print(F("Temperatura: "));
        Serial.print((temperatura+temperatura2)/2); //Aqui é feita a média da temperatura obtida nos sensores DHT11 e BMP180
        Serial.print(F("°C "));
        Serial.print(F("/ Umidade: "));
        Serial.print(umidade);
        Serial.print(F("% "));
        Serial.print(F("/ Pressão atmosférica: "));
        Serial.print(pressao);
        Serial.print(F(" KPa "));
        Serial.println();
        Serial.print(F("/ Altitude aproximada: "));
        Serial.print(altitude);
        Serial.print(F(" metros "));
        Serial.print(F(" / Intensidade UV: "));
        Serial.print(uvInt);
        Serial.print(F(" MJ/m² por hora "));
        Serial.print(F("/ Umidade do solo: "));
        String umidadeSolo;
        if(umidsolo>2000) umidadeSolo = "Baixa";
        else if(umidsolo>1000) umidadeSolo = "Média";
        else if(umidsolo>0) umidadeSolo = "Alta";
        Serial.println(umidadeSolo);
        //Mostrar número do dia do ano
        J = calcularDiaDoAno(date.day, date.month, date.year);
        Serial.print("Dia do ano (J): ");
        Serial.print(J);
        Serial.println();



        numLeituras++;
        

        String payload = "{\"d\":{\"num\":";                 // Inicia Payload
        payload += numLeituras;
        payload += ",\"data\":";
        payload += "\""; payload+= date.year; payload+= "/"; 
        if(date.month<10){ payload+= "0"; payload += date.month;} else payload += date.month;
        payload+= "/"; 
        if(date.day<10){ payload+= "0"; payload += date.day;} else payload+= date.day; 
        payload+= " - "; 
        if(date.hours<10){ payload+= "0"; payload += date.hours;} else payload+= date.hours; 
        payload+= ":"; 
        if(date.minutes<10){ payload+= "0"; payload += date.minutes;} else payload+= date.minutes; 
        payload+= "\"";
        payload += ",\"temperatura\":";
        payload += ((temperatura+temperatura2)/2);
        payload += ",\"umidade\":";
        payload += umidade;
        payload += ",\"pressao\":";
        payload += pressao;
        payload += ",\"altitude\":";
        payload += altitude;
        payload += ",\"uv\":";
        payload += uvInt2;
        payload += ",\"umidSolo\":"; payload += "\"";
        payload += umidadeSolo; payload += "\"";  

        if(date.hours==23){
          tempMedia /= numLeituras;
          umidMedia /= numLeituras;
          
          payload += ",\"tempMedia\":";
          payload += tempMedia;
          tempMedia = 0;                                     
          
          payload += ",\"umidMedia\":";
          payload += umidMedia;   
          umidMedia = 0;                                   
          
          payload += ",\"uvDia\":";
          payload += uvIntDia;                                     
          
          numLeituras = 0;

          delta1 = calcDelta(tempMedia);
          gama1 = calcGama(pressao);
          es1 = calcEs(tempMedia);
          ea1 = calcEa(umidMedia, es1);
          rn1 = calcRn(0.22, uvIntDia, tempMax, tempMin, ea1, altitude, J, latitude);
          EToPMF = calcETo(delta1, rn1, gama1, es1, ea1, tempMedia, 2);
          
          payload += ",\"eto\":";
          payload += EToPMF;
          
          uvIntDia = 0;
          
          Kc = kcMilho(150, true, diadociclo);
          Serial.print("Dia do ciclo: ");
          Serial.println(diadociclo);
          Serial.print("Kc: ");
          Serial.println(Kc);
          
          payload += ",\"kc\":";
          payload += Kc;
          
          diadociclo++;

          ETc = EToPMF * Kc;
          
          payload += ",\"etc\":";
          payload += ETc;                                     
          delay(100);

          ETca+=ETc;

          payload += ",\"etca\":";
          payload += ETca;                                     

          payload += ",\"irrig\":";
          if((umidsolo>2000)) {
            Serial.println(" Umidade do solo baixa. IRRIGAR!");
            payload += "\" \"";
            Serial.print(" Quantidade a ser irrigada: ");
            Serial.print(ETca);
            Serial.print(" mm");
            ETca = 0;
          }else {
            payload += "\"Não\"";
          }
        }
        payload += "}}";
        Serial.println(payload);
        client.publish(eventTopic, (char*) payload.c_str() );// Publica a Payload
        delay(100);                                          
       
        
        
        

        /*float delta1 = calcDelta(25.6);
        Serial.print("Delta: ");
        Serial.print(delta1);
        float gama1 = calcGama(97.402);
        Serial.print("Gama: ");
        Serial.print(gama1);
        float es1 = calcEs(25.6);
        Serial.print("Es: ");
        Serial.print(es1);
        float ea1 = calcEa(81.6, es1);
        Serial.print("Ea: ");
        Serial.print(ea1);
        //float latitude = -12.1483; //Latitude da cidade de Barreiras-BA
        //float rn1 = calcRn(0.22, uvIntDia, tempMax, tempMin, ea1, altitude, J, latitude);
        float EToPMF = calcETo(delta1, 11.7, gama1, es1, ea1, 25.6, 1.6);
        Serial.println();
        Serial.print("Evapotranspiração de referência: ");
        Serial.print(EToPMF);
        Serial.println(" mm/dia ");
        Serial.println();

        float rn1 = calcRn(0.23, 17.6, 32.3, 22.3, ea1, 335, 288, -20.3972975);
        Serial.print("Rn: ");
        Serial.print(rn1);
        */

        //---------------------------------------------------------------------------------------------------------------

        
        Serial.println();
        delay(200);
        
    }    

    delay(2000);
}
