#include <SPI.h>
#include <WiFiNINA.h>
#include <Arduino_LSM6DSOX.h>
#include <WiFiUdp.h>

///////variables de la conexion
char ssid[] = "guante";        // your network SSID (name)
char pass[] = "12345678";      // your network password (use for WPA, or use as key for WEP)
//int keyIndex = 0;                 // your network key index number (needed only for WEP)
int status = WL_IDLE_STATUS;

WiFiUDP Udp;
IPAddress Ordenador; //cambia cada vez
int puerto=1001;

int dedo1=A0;
int dedo2=A1;
int dedo3=A2;
int dedo4=A3;
int flag=0;

#define UDP_TX_PACKET_MAX_SIZE 40

void setup() {
  randomSeed(analogRead(0));

  Serial.begin(9600);   // initialize serial communication

  pinMode(LEDR, OUTPUT);  //inicializa leds para saber estado
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  digitalWrite(LEDR, HIGH);

  /*pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);*/


  while (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    delay(100);
  }
  while (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    delay(100);
  }

   String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  delay(1000);
  digitalWrite(LEDG, HIGH);
//Conectarse a la red wifi
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to network: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(2000);
  }

  Serial.println("You're connected to the network");
  //Conectarse al cliente
  Udp.begin(puerto);

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
  digitalWrite(LEDR, LOW);

}

void loop() {
  int medidas[4];  
  Serial.print("Pon la posicion: ");
  flag=random(10);
  Serial.println(flag);  
  delay(3700);

  medidas[0]=analogRead(dedo1);
  medidas[1]=analogRead(dedo2);  
  medidas[2]=analogRead(dedo3);
  medidas[3]=analogRead(dedo4);
  char data[9];
  for (int i=0;i<4;i++){
    data[i*2]=medidas[i]%128;
    data[i*2+1]=medidas[i]/128;
    Serial.print(medidas[i]);
    Serial.print("  ");  
    /*Serial.print(data[i*2]);   
    Serial.print("  ");  
    Serial.print(data[i*2+1]);
    Serial.print("  ");    
    Serial.print(medidas[i]%128);
    Serial.print("  ");
    Serial.println(medidas[i]/128);*/   
  }
  Serial.println();
  data[8]=flag;






//Envio de datos
    Udp.beginPacket(Ordenador, puerto);
    Udp.write(data,sizeof(data));
    Udp.endPacket();

}
