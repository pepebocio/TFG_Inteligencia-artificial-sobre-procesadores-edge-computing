#include <Arduino_LSM6DSOX.h>

unsigned long oldmillis = 0;
const long sensorinterval = 20;

float thetaG = 0;
float phiG = 0;
float yawG = 0;

float oldx = 0;
float oldy = 0;
float oldz = 0;

float xoff=0;
float yoff=0;
float zoff=0;
//190000 para 30 mins
//30000 para 5 mins
int numeromedidas=30000;
void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.print("Gyroscope sample rate = ");  
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println("Hz");
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");
  int i=0;
  float x, y, z;
  while (i<numeromedidas){
    if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);
    xoff+=x;
    yoff+=y;
    zoff+=z;
    i++;
  }
  }
  xoff=xoff/numeromedidas;
  yoff=yoff/numeromedidas;
  zoff=zoff/numeromedidas;
  Serial.println(xoff);
  Serial.println(yoff);
  Serial.println(zoff);
}

void loop() {

}
