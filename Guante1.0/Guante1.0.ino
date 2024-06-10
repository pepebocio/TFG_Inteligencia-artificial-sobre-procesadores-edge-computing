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


/////Variables para el calibrado de pantalla
int timer=0;
int counter=0;
int tiempocalibrado=3000;


/////Indica si el giroscopo esta activo
bool midiendoAngulo=false;


/////variables para comprobar las ultimas posiciones de la mano
const int numeroCasos = 10; // Número de mediciones a almacenar
int ultimosCasos[numeroCasos]; // Array para almacenar las últimas mediciones
int casoIndex = 0; // Índice para rastrear la posición actual en el array


/////Variable para controlar la duración del bucle
int tiempoBucle=0;

/////¿Sobra?
int patron[4]={0,0,0,0};

void setup() {
/////inicializa led y puerto serie y el led que nos avisa del estado de conexión
  Serial.begin(115200); 
  pinMode(LEDR, OUTPUT);  
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  colorLed(2);


/////Inicializa la red
  tf.setNumInputs(4);
  tf.setNumOutputs(10);
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



/////Selecciona el IP para enviar broadcast
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


  
/////Envia el paquete de incio de comunicación
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

      /////Codificar los datos
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
      /////Envio de datos
        Udp.beginPacket(Ordenador, puerto);
        Udp.write(datos);
        Udp.endPacket();
        timer=millis();
        counter++;
        Serial.println(counter);              
      }
  }

/////Calibra los valores máximos de la mano
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



/////Calibra los valores mínimos de la mano

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


  /////Toma los primeros valores para configurar la ultimas posiciones de la mano
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

}


 

void loop() {
  tiempoBucle=micros();
  int medidas[4];
  float medidasNorm[4];

/////Mide las posiciones de los dedos y las normaliza
  medidas[0]=analogRead(dedo1);
  medidas[1]=analogRead(dedo2);  
  medidas[2]=analogRead(dedo3);
  medidas[3]=analogRead(dedo4);
  for(int i=0;i<4;i++){
    medidasNorm[i]=normalizar(medidas[i],dedosmin[i], dedosdif[i]);
    //Serial.print(medidasNorm[i]);
    //Serial.print("  ");
  }

/////Asigna una posición usando la red neuronal

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
  //Serial.print(casoActual);
  //Serial.print("  ");

/////Guarda el ultimo caso y comprueba si todos los ultimos han sido iguales
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



/////varible para saber cuando empiezan los casos con giroscopo

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



///// Activo el giroscopo
  if(caso==2||caso==3||caso==4||caso==5){      //casos que necesitan el giroscopo
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
    //Serial.println(beta);

    /////Codifico el paquete
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
    //Envio de datos
    Udp.beginPacket(Ordenador, puerto);
    Udp.write(datos);
    Udp.endPacket();
    /*Serial.print(int((alpha+128)*4));
    Serial.print("  ");
    Serial.println(int((beta+128)*4));
    printBinary(datos[0]);
    printBinary(datos[1]),
    printBinary(datos[2]);*/           

  }
  
  /////Envio solo la posición si no estoy usando el giroscopo
  else{
    char CasoChar=caso+1;
    Udp.beginPacket(Ordenador, puerto);
    Udp.write(CasoChar);
    Udp.endPacket();
  }
  //Serial.println(caso);

  /////si recibo algo significa que el ordenador pide recalibrar la extensión de la mano
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

  /////Control de duración del bucle
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