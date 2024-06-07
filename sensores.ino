#include <QTRSensors.h>

#define NUM_SENSORS             8  // número de sensores utilizados
#define NUM_SAMPLES_PER_SENSOR  4  // promedio de 4 muestras analógicas por lectura de sensor
#define EMITTER_PIN             9  // pin digital que controla el emisor

// Los sensores 0 a 5 están conectados a los pines analógicos 0 a 5, respectivamente
QTRSensorsAnalog qtra((unsigned char[]) {7, 6, 5, 4, 3, 2, 1, 0}, 
  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];


void setup()
{
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // encender el LED de Arduino para indicar que estamos en modo de calibración
  for (int i = 0; i < 400; i++)  // calibrar durante aproximadamente 10 segundos
  {
    qtra.calibrate();       // leer todos los sensores 10 veces a 2.5 ms por seis sensores (aprox. 25 ms por llamada)
  }
  digitalWrite(13, LOW);     // apagar el LED de Arduino para indicar que hemos terminado con la calibración

  // imprimir los valores mínimos de calibración medidos cuando los emisores estaban encendidos
  Serial.begin(9600);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  // imprimir los valores máximos de calibración medidos cuando los emisores estaban encendidos
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}


void loop()
{
  unsigned int position = qtra.readLine(sensorValues);
  
  // imprimir los valores del sensor como números de 0 a 1000, donde 0 significa reflectancia máxima y
  // 1000 significa reflectancia mínima, seguido de la posición de la línea
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  //Serial.println(); // descomenta esta línea si estás usando valores sin procesar
  Serial.println(position); // comenta esta línea si estás usando valores sin procesar
  
  delay(50);
}
