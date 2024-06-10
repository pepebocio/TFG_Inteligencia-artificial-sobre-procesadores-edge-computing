//En este programa cambiamos GiroUDPprecision para coordinar la comunicacion con el ordenador

#include <SPI.h>
#include <WiFiNINA.h>
#include <Arduino_LSM6DSOX.h>
#include <WiFiUdp.h>
#include <eloquent_tensorflow_cortexm.h>
#include "RedDef.h"


/////Inicializacion de la red
#define NUM_OPS 10
#define ARENA_SIZE 6*1024
using Eloquent::CortexM::TensorFlow;
TensorFlow<NUM_OPS, ARENA_SIZE> tf;
int dedo1=A0;
int dedo2=A1;
int dedo3=A2;
int dedo4=A3;
const int inputs=4;
const int outputs=10;

int caso;

//////Medidas de tiempos
const int numerotiempos=5000;
int tiempoADC[numerotiempos];
int tiempoRED[numerotiempos];
int tiempoGIRO[numerotiempos];
int tiempoWIFI[numerotiempos];
int tiempoBUCLE[numerotiempos];
int tiempoCODE[numerotiempos];
int indicetiempo=0;

/////Valores extension mano
double dedosmax[4]={592,609,620,620};
double dedosmin[4]={366,381,387,351};
double dedosdif[4];


/////variables de la conexion
const char ssid[] = "guante";      
const char pass[] = "12345678";      
int status = WL_IDLE_STATUS;
WiFiUDP Udp;
IPAddress Ordenador; //cambia cada vez
const int puerto=1002;


/////Variable del giroscopo
unsigned long t1 = 0;
long sensorinterval = 0;
float alpha = 0;
float titarad=0;
float beta = 0;


////Offset Acelerometro
float xoff=0.39;
float yoff=-0.35;
float zoff=-0.36;


/////Calibrado pantalla
int timer=0;
int counter=0;
int tiempocalibrado=300;

bool midiendoAngulo=false;

const int numeroCasos = 10; // Número de mediciones a almacenar
int ultimosCasos[numeroCasos]; // Array para almacenar las últimas mediciones
int casoIndex = 0; // Índice para rastrear la posición actual en el array

int tiempoBucle=0;

int patron[4]={0,0,0,0};

void setup() {
/////inicializa led y puerto serie


  Serial.begin(115200);   // initialize serial communication
  pinMode(LEDR, OUTPUT);  //inicializa leds para saber estado
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  colorLed(2);

/////Inicializa la red
  tf.setNumInputs(4);
  tf.setNumOutputs(10);
  // add required ops
  tf.resolver.AddFullyConnected();
  while (!tf.begin(modelo_tflite).isOk()){
    Serial.println(tf.exception.toString()); 
  } 


/////comprueba la IMU
  while (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    delay(100);
  }
/////COmprueba el WIFI NINA
  while (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    delay(100);
  }
/////Comprueba el firmware del wifi
   String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

delay(100);
colorLed(5);


//Conectarse a la red wifi
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to network: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(1000);
  }
colorLed(3);
Serial.println("You're connected to the network");
  //Conectarse al cliente
  Udp.begin(puerto);



/////busca el ip para enviar broadcast
byte octetos[4];
IPAddress ip = WiFi.localIP();
Serial.println(ip);
  // Obtener los octetos de la IP original
  octetos[0] = ip[0];
  octetos[1] = ip[1];
  octetos[2] = ip[2];
  // Eliminar el último octeto
  octetos[3] = 255;
  // Crear la nueva IPAddress
  Ordenador = IPAddress(octetos);
  Serial.println(Ordenador);


  

  char inicio=1;
  Udp.beginPacket(Ordenador, puerto);
    Udp.write(inicio);
  Udp.endPacket();
  timer=millis();

  t1= micros();
/////Calibrar esquinas
  while(counter<4){
      float x, y, z;
      float ax, ay, az;
      float a;

      //Obtener los angulos del acelerometro
      if(IMU.accelerationAvailable()) {//Seguramente por velocidad de envio sobre la condición, por si eso mejorara algo
        IMU.readAcceleration(ax, ay, az);    
        ax+=0.025;
        ay+=0.01;
        az+=0.01+0.01*abs(ay);
        a=sqrt((ax*ax)+(ay*ay)+(az*az));
        alpha=57.2958*asin(ay/a);
        titarad=-atan(az/ax);
        if(ax>0)titarad+=3.1416;
      }

      //Obtener los angulos del giroscopo
      float sina=cos(alpha*0.017453); 
        IMU.readGyroscope(x, y, z);
        sensorinterval= micros()-t1;
        if((x-xoff)>0.7){
          beta+=cos(titarad)*1.209*((x -xoff) * (sensorinterval / 1000000.))/sina;
        }
        else{if(-(x-xoff)>0.7){
          beta+=cos(titarad)*1.219*((x -xoff) * (sensorinterval / 1000000.))/sina;
        }}
          
        if((z-zoff)>0.7){
          beta+=-sin(titarad)*1.187*((z-zoff) * (sensorinterval / 1000000.))/sina;
        }
        else{if(-(z-zoff)>0.7){
          beta+=-sin(titarad)*1.187*((z-zoff) * (sensorinterval / 1000000.))/sina;
        }}
      t1=micros();

      if((millis()-timer)>tiempocalibrado && counter<4 /*&& int((alpha+128)*4)!=512*/){
      char datos[3];
      bool PaqueteVacio=true;
      while(PaqueteVacio){
        datos[0] = int((alpha+128)*4)  & 0xFF; // Los 8 bits menos significativos de valor1
        datos[1] = ((int((alpha+128)*4) >> 8) & 0x03) | ((int((beta+128)*4) & 0x3F) << 2); // 2 bits más significativos de valor1 + 6 bits de valor2
        datos[2] = ((int((beta+128)*4) >> 6) & 0x0F); // Los 4 bits menos significativos de valor2
        PaqueteVacio=false;
        for(int i=0;i++;i<sizeof(datos)){
          if(datos[i]==0x00){
            alpha++;
            beta++;
            PaqueteVacio=true;
            break;
          }
        }
      }
      //Envio de datos
        Udp.beginPacket(Ordenador, puerto);
        Udp.write(datos);
        Udp.endPacket();
        timer=millis();
        counter++;
        Serial.println(counter);              
      }
  }

/////Valores máximos de la mano
  Serial.println("Amplitud mano");
  delay(1000);
  int margen=0;
  int manoMaximo[4]={0,0,0,0};
  int MedidasMargen=10;
  while(margen<4*MedidasMargen){
    int medidas[4];

    medidas[0]=analogRead(dedo1);
    medidas[1]=analogRead(dedo2);  
    medidas[2]=analogRead(dedo3);
    medidas[3]=analogRead(dedo4);

    for(int i=0;i<4;i++){
      if(medidas[i]>manoMaximo[i]){
        //Serial.println();
        //Serial.print(i);
        //Serial.print("  ");
        //delay(2000);
        //Serial.println(medidas[i]);
        manoMaximo[i]=medidas[i];
        /*for(int i=0;i<4;i++){
          Serial.print(manoMaximo[i]);
          Serial.print("  ");
        }*/
        margen=0;
      }
      else margen++;

    }     
    Serial.print("Margen: ");
    Serial.println(margen);
    delay(200);

  }




  char aviso=1;
  Udp.beginPacket(Ordenador, puerto);
  Udp.write(aviso);
  Udp.endPacket();



/////Valores minimos de la mano

  delay(1000);
  margen=0;
  int manoMinimo[4];

  for(int i=0;i<4;i++){
    manoMinimo[i]=manoMaximo[i];
  }
  while(margen<4*MedidasMargen){
    int medidas[4];

    medidas[0]=analogRead(dedo1);
    medidas[1]=analogRead(dedo2);  
    medidas[2]=analogRead(dedo3);
    medidas[3]=analogRead(dedo4);

    for(int i=0;i<4;i++){
      if(medidas[i]<manoMinimo[i]){
        /*Serial.println();
        Serial.print(i);
        Serial.print("  ");
        delay(2000);
        Serial.println(medidas[i]);*/
        manoMinimo[i]=medidas[i];
        /*for(int i=0;i<4;i++){
          Serial.print(manoMinimo[i]);
          Serial.print("  ");
        }*/
        margen=0;
      }
      else margen++;
    }
    Serial.print("Margen: ");
    Serial.println(margen);
    delay(200);

  }

  for(int i=0;i<4;i++){
    dedosmax[i]=manoMaximo[i];
    dedosmin[i]=manoMinimo[i];
    dedosdif[i]=dedosmax[i]-dedosmin[i];
  }

  Udp.beginPacket(Ordenador, puerto);
  Udp.write(aviso);
  Udp.endPacket();

  for(int i=0;i<numeroCasos;i++){
    int medidas[4];
    float medidasNorm[4];

    medidas[0]=analogRead(dedo1);
    medidas[1]=analogRead(dedo2);  
    medidas[2]=analogRead(dedo3);
    medidas[3]=analogRead(dedo4);
    for(int i=0;i<4;i++){
      medidasNorm[i]=normalizar(medidas[i],dedosmin[i], dedosdif[i]);
      Serial.print(medidas[i]);
      Serial.print("  ");
    }

    while (!tf.predict(medidasNorm).isOk())
    Serial.println(tf.exception.toString());

    /*char data[9];
    for (int i=0;i<4;i++){
      data[i*2]=medidas[i]%128;
      data[i*2+1]=medidas[i]/128;
      Serial.print(medidas[i]);
      Serial.print("  ");  
    }*/
    int casoActual=0;
    float maximo=0;
    for(int i=0;i<10;i++) {
      //Serial.print(tf.result(i));
      //Serial.print("  ");
        if(tf.result(i)>maximo) { 
          maximo=tf.result(i);
          casoActual=i;
      }
    }
    ultimosCasos[casoIndex] = casoActual;
    casoIndex = (casoIndex + 1) % numeroCasos;
  }


  delay(1000);
  colorLed(0);
  delay(4000);

}


 

void loop() {
  tiempoBucle=micros();
  
  int medidas[4];
  float medidasNorm[4];
  int tiempo=micros();
  medidas[0]=analogRead(dedo1);
  Serial.print(micros()-tiempo);
  medidas[1]=analogRead(dedo2);  
  medidas[2]=analogRead(dedo3);
  medidas[3]=analogRead(dedo4);
  for(int i=0;i<4;i++){
    medidasNorm[i]=normalizar(medidas[i],dedosmin[i], dedosdif[i]);
    //Serial.print(medidasNorm[i]);
    //Serial.print("  ");
  }
  tiempoADC[indicetiempo]=micros()-tiempo;
  tiempo=micros();

  while (!tf.predict(medidasNorm).isOk())
  Serial.println(tf.exception.toString());

  

  /*char data[9];
  for (int i=0;i<4;i++){
    data[i*2]=medidas[i]%128;
    data[i*2+1]=medidas[i]/128;
    Serial.print(medidas[i]);
    Serial.print("  ");  
  }*/
  int casoActual=0;
  float maximo=0;
  for(int i=0;i<10;i++) {
    //Serial.print(tf.result(i));
    //Serial.print("  ");
      if(tf.result(i)>maximo) { 
        maximo=tf.result(i);
        casoActual=i;
    }
  }
  
  tiempoRED[indicetiempo]=micros()-tiempo;
  Serial.println(casoActual);
  //Serial.print("  ");

  ultimosCasos[casoIndex] = casoActual;
  casoIndex = (casoIndex + 1) % numeroCasos; // Avanza al siguiente índice, circularmente

/////Solo actualiza la posicion de la mano si se ha repetido varias veces
  bool todasIguales = true;
  for (int i = 1; i < numeroCasos; i++) {
    if (ultimosCasos[i] != ultimosCasos[0]) {
      todasIguales = false;
      break;
    }
  }
  if (todasIguales) {
    caso=ultimosCasos[0];
  }



/////varible para saber cuando empiezo los casos con giroscopo

  if((caso==2||caso==3||caso==4||caso==5)&& !midiendoAngulo){
    beta=0;
    t1=micros(); 
    midiendoAngulo=true;
    //Serial.print("Activo giroscopo");
    //delay(2000);
  }
  if(!(caso==2||caso==3||caso==4||caso==5||caso==1)&& midiendoAngulo){
    midiendoAngulo=false;
    //Serial.println("Descativo Giroscopo");
    //delay(2000);
  }




  if(caso==2||caso==3||caso==4||caso==5){      //casos que necesitan el giroscopo
    tiempo=micros();
    float x, y, z;
    float ax, ay, az;
    float a, provAlpha;

    //Obtener los angulos del acelerometro
    if(IMU.accelerationAvailable()) {//Seguramente por velocidad de envio sobre la condición, por si eso mejorara algo
      IMU.readAcceleration(ax, ay, az);    
      ax+=0.025;
      ay+=0.01;
      az+=0.01+0.01*abs(ay);
      a=sqrt((ax*ax)+(ay*ay)+(az*az));
      provAlpha=57.2958*asin(ay/a);
      if((alpha-provAlpha)<-0.2||(alpha-provAlpha)>0.2){
        alpha=provAlpha;
      }

      titarad=-atan(az/ax);
      if(ax>0)titarad+=3.1416;
      //Serial.println(provAlpha);
      //Serial.println(alpha);
    }

    //Obtener los angulos del giroscopo
    float sina=cos(alpha*0.017453); 
      IMU.readGyroscope(x, y, z);
      sensorinterval= micros()-t1;
      /*Serial.print(x-xoff);
      Serial.print("  ");
      Serial.println(z-zoff);*/
      if((x-xoff)>1){
        beta+=cos(titarad)*1.209*((x -xoff) * (sensorinterval / 1000000.))/sina;
        //Serial.print("suma X  ");
      }
      else{if(-(x-xoff)>1){
        beta+=cos(titarad)*1.219*((x -xoff) * (sensorinterval / 1000000.))/sina;
        //Serial.print("resta X ");
      }}
        
      if((z-zoff)>1){
        beta+=-sin(titarad)*1.187*((z-zoff) * (sensorinterval / 1000000.))/sina;
        //Serial.print("suma Z  ");
      }
      else{if(-(z-zoff)>1){
        beta+=-sin(titarad)*1.187*((z-zoff) * (sensorinterval / 1000000.))/sina;
        //Serial.print("Resta Z ");
      }}
    t1=micros();
    tiempoGIRO[indicetiempo]=micros()-tiempo;
    //Serial.println(beta);

    tiempo=micros();
    char datos[4];
    bool PaqueteVacio=true;
    while(PaqueteVacio==true){
      char CasoChar=caso+1;
      datos[0] = CasoChar;
      datos[1] = int((alpha+128)*4)  & 0xFF; // Los 8 bits menos significativos de valor1
      datos[2] = ((int((alpha+128)*4) >> 8) & 0x03) | ((int((beta+128)*4) & 0x3F) << 2); // 2 bits más significativos de valor1 + 6 bits de valor2
      datos[3] = ((int((beta+128)*4) >> 6) & 0x0F); // Los 4 bits menos significativos de valor2
      PaqueteVacio=false;
      for(int i=0;i<sizeof(datos);i++){
        //printBinary(datos[i]);
        if(datos[i]==0x00){
          alpha+=0.25;
          beta+=0.25;
          PaqueteVacio=true;
          break;
        }
      }
    }
    tiempoCODE[indicetiempo]=micros()-tiempo;
    tiempo=micros();
    //Envio de datos
    Udp.beginPacket(Ordenador, puerto);
    Udp.write(datos);
    Udp.endPacket();
    tiempoWIFI[indicetiempo]=micros()-tiempo;

    /*Serial.print(int((alpha+128)*4));
    Serial.print("  ");
    Serial.println(int((beta+128)*4));
    printBinary(datos[0]);
    printBinary(datos[1]),
    printBinary(datos[2]);*/           

  }
  
  else{
    char CasoChar=caso+1;
    Udp.beginPacket(Ordenador, puerto);
    Udp.write(CasoChar);
    Udp.endPacket();
  }
  //Serial.println(caso);

  if(Udp.parsePacket()){
    Serial.println("calibrar");
    char calibrado='a';
    delay(500);
    Udp.beginPacket(Ordenador, puerto);
    Udp.write(calibrado);
    Udp.endPacket();
    /////Valores máximos de la mano
    Serial.println("Amplitud mano");
    delay(500);
    int margen=0;
    int manoMaximo[4]={0,0,0,0};
    int MedidasMargen=10;
    while(margen<4*MedidasMargen){
      int medidas[4];

      medidas[0]=analogRead(dedo1);
      medidas[1]=analogRead(dedo2);  
      medidas[2]=analogRead(dedo3);
      medidas[3]=analogRead(dedo4);

      for(int i=0;i<4;i++){
        if(medidas[i]>manoMaximo[i]){
          //Serial.println();
          //Serial.print(i);
          //Serial.print("  ");
          //delay(2000);
          //Serial.println(medidas[i]);
          manoMaximo[i]=medidas[i];
          /*for(int i=0;i<4;i++){
            Serial.print(manoMaximo[i]);
            Serial.print("  ");
          }*/
          margen=0;
        }
        else margen++;

      }     
      Serial.print("Margen: ");
      Serial.println(margen);
      delay(200);

    }




    char aviso=1;
    Udp.beginPacket(Ordenador, puerto);
    Udp.write(aviso);
    Udp.endPacket();



    /////Valores minimos de la mano

    delay(1000);
    margen=0;
    int manoMinimo[4];

    for(int i=0;i<4;i++){
      manoMinimo[i]=manoMaximo[i];
    }
    while(margen<4*MedidasMargen){
      int medidas[4];

      medidas[0]=analogRead(dedo1);
      medidas[1]=analogRead(dedo2);  
      medidas[2]=analogRead(dedo3);
      medidas[3]=analogRead(dedo4);

      for(int i=0;i<4;i++){
        if(medidas[i]<manoMinimo[i]){
          /*Serial.println();
          Serial.print(i);
          Serial.print("  ");
          delay(2000);
          Serial.println(medidas[i]);*/
          manoMinimo[i]=medidas[i];
          /*for(int i=0;i<4;i++){
            Serial.print(manoMinimo[i]);
            Serial.print("  ");
          }*/
          margen=0;
        }
        else margen++;
      }
      Serial.print("Margen: ");
      Serial.println(margen);
      delay(200);

    }

    for(int i=0;i<4;i++){
      dedosmax[i]=manoMaximo[i];
      dedosmin[i]=manoMinimo[i];
      dedosdif[i]=dedosmax[i]-dedosmin[i];
    }

    Udp.beginPacket(Ordenador, puerto);
    Udp.write(aviso);
    Udp.endPacket();
    delay(1000);
    for(int i=0;i<4;i++){
      patron[i]=0;
    }

  }


  tiempoBUCLE[indicetiempo]=(micros()-tiempoBucle);
  indicetiempo++;

  if(indicetiempo==numerotiempos){
    float mediaRed=0;
    float mediaBucle=0;
    float mediaWifi=0;
    float mediaCode=0;
    float mediaGiro=0;
    float mediaAdc=0;
    for(int i=0;i<numerotiempos;i++){
      mediaRed+=tiempoRED[i];
      mediaBucle+=tiempoBUCLE[i];
      mediaWifi+=tiempoWIFI[i];
      mediaCode+=tiempoCODE[i];
      mediaGiro+=tiempoGIRO[i];
      mediaAdc+=tiempoADC[i];
    }
    mediaRed=mediaRed/numerotiempos;
    mediaBucle=mediaBucle/numerotiempos;
    mediaWifi=mediaWifi/numerotiempos;
    mediaCode=mediaCode/numerotiempos;
    mediaGiro=mediaGiro/numerotiempos;
    mediaAdc=mediaAdc/numerotiempos;


    float sigmaRed=0;
    float sigmaBucle=0;
    float sigmaWifi=0;
    float sigmaCode=0;
    float sigmaGiro=0;
    float sigmaAdc=0;
    for(int i=0;i<numerotiempos;i++){
      sigmaRed+=(tiempoRED[i]-mediaRed)*(tiempoRED[i]-mediaRed);
      sigmaBucle+=(tiempoBUCLE[i]-mediaBucle)*(tiempoBUCLE[i]-mediaBucle);
      sigmaWifi+=(tiempoWIFI[i]-mediaWifi)*(tiempoWIFI[i]-mediaWifi);
      sigmaCode+=(tiempoCODE[i]-mediaCode)*(tiempoCODE[i]-mediaCode);
      sigmaGiro+=(tiempoGIRO[i]-mediaGiro)*(tiempoGIRO[i]-mediaGiro);
      sigmaAdc+=(tiempoADC[i]-mediaAdc)*(tiempoADC[i]-mediaAdc);
    }
    sigmaRed=sqrt(sigmaRed/(numerotiempos-1));
    sigmaBucle=sqrt(sigmaBucle/(numerotiempos-1));
    sigmaWifi=sqrt(sigmaWifi/(numerotiempos-1));
    sigmaCode=sqrt(sigmaCode/(numerotiempos-1));
    sigmaGiro=sqrt(sigmaGiro/(numerotiempos-1));
    sigmaAdc=sqrt(sigmaAdc/(numerotiempos-1));   

    Serial.print("RED:  ");
    Serial.print(mediaRed);
    Serial.print("+-");
    Serial.println(sigmaRed);

    Serial.print("Bucle:  ");
    Serial.print(mediaBucle);
    Serial.print("+-");
    Serial.println(sigmaBucle);

    Serial.print("Wifi:  ");
    Serial.print(mediaWifi);
    Serial.print("+-");
    Serial.println(sigmaWifi);

    Serial.print("Code:  ");
    Serial.print(mediaCode);
    Serial.print("+-");
    Serial.println(sigmaCode);

    Serial.print("Giro:  ");
    Serial.print(mediaGiro);
    Serial.print("+-");
    Serial.println(sigmaGiro);

    Serial.print("ADC:  ");
    Serial.print(mediaAdc);
    Serial.print("+-");
    Serial.println(sigmaAdc);

    delay(1000000);
  }


  if(10000>(micros()-tiempoBucle))
    delayMicroseconds(10000-(micros()-tiempoBucle));

}


void printBinary(char c) {
  for (int i = 7; i >= 0; --i) {
    Serial.print((c >> i) & 0x01);
  }
  Serial.println();
}

float normalizar(float x, double xmin, double xdif){
  float norm=(x-xmin)/xdif;
  if(norm<0) return 0;
  if(norm>1) return 1;
  else return norm;
}

int maximo (float datos[], int len){
  float maximo=0;
  int indice=0;
  for(int i=0; i<len; i++){
    if(datos[i]>maximo) {
      maximo=datos[i];
      indice=i;
    }
  }
  return indice;
}

void colorLed (int i){
  switch (i){
    case 0://apagado
      digitalWrite(LEDR,LOW);
      digitalWrite(LEDG,LOW);
      digitalWrite(LEDB,LOW);
      break;
    case 1://blanco
      digitalWrite(LEDR,HIGH);
      digitalWrite(LEDG,HIGH);
      digitalWrite(LEDB,HIGH);
      break;
    case 2://rojo
      digitalWrite(LEDR,HIGH);
      digitalWrite(LEDG,LOW);
      digitalWrite(LEDB,LOW);
      break;
    case 3://verde
      digitalWrite(LEDR,LOW);
      digitalWrite(LEDG,HIGH);
      digitalWrite(LEDB,LOW);
      break;
    case 4://azul
      digitalWrite(LEDR,LOW);
      digitalWrite(LEDG,LOW);
      digitalWrite(LEDB,HIGH);
      break;
    case 5://amarillo
      digitalWrite(LEDR,HIGH);
      digitalWrite(LEDG,HIGH);
      digitalWrite(LEDB,LOW);
      break;
    case 6://rosa
      digitalWrite(LEDR,HIGH);
      digitalWrite(LEDG,LOW);
      digitalWrite(LEDB,HIGH);
      break;
    case 7://azul claro
      digitalWrite(LEDR,LOW);
      digitalWrite(LEDG,HIGH);
      digitalWrite(LEDB,HIGH);
      break;
  }
  return;
}