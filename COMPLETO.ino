#include <QTRSensors.h>

// Definición de pines
#define btn1      9   // Pin para el botón 1
#define btn2      9  // Pin para el botón 2
#define led1      13  // Pin para el LED

#define led_on    9   // Pin para encender el LED
#define mi1       5   // Pin para el motor izquierdo
#define mi2       4   // Pin para el motor izquierdo
#define pwmi      3   // Pin PWM para el motor izquierdo
#define md1       6   // Pin para el motor derecho
#define md2       7   // Pin para el motor derecho
#define pwmd      11   // Pin PWM para el motor derecho

// Parámetros para los sensores QTR
#define NUM_SENSORS             8   // Número de sensores utilizados
#define NUM_SAMPLES_PER_SENSOR  1   // Número de muestras por sensor
#define EMITTER_PIN             9   // Pin de control del emisor

// Inicialización del objeto QTRSensorsAnalog
QTRSensorsAnalog qtra((unsigned char[]) {7, 6, 5, 4, 3, 2,1,0},NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

// Variables para el control PID
int proporcional=0;
int derivativo=0;
int integral=0;
int salida_pwm=0;
int proporcional_pasado=0;
int position=0;

// Parámetros PID
int velocidad=120;
float KP=0.01, KD=0.15 , KI=0.0001; // Parámetros de control PID

// Parámetros para el sensado de la línea
int linea=0; 
int flanco_color=  0 ;
int en_linea=  500 ;
int ruido= 30;

// Definición de pines para botones
int boton1=9;
int boton2=9;

void setup()
{
    // Configuración de pines
    pinMode(led1,OUTPUT);
    pinMode(led_on,OUTPUT);
    pinMode(mi1,OUTPUT);
    pinMode(mi2,OUTPUT);
    pinMode(pwmi,OUTPUT);
    pinMode(md1,OUTPUT);
    pinMode(md2,OUTPUT);
    pinMode(pwmd,OUTPUT);

    // Calibración de los sensores
    digitalWrite(led1, HIGH);    // Encender el LED para indicar que estamos en modo de calibración
    for (int i = 0; i < 200; i++)  // Calibrar durante aproximadamente 10 segundos
    {
        qtra.calibrate();       // Leer todos los sensores 10 veces
    }
    digitalWrite(led1, LOW);     // Apagar el LED para indicar que hemos terminado con la calibración

    // Esperar hasta que se presione el botón 2 para continuar
    while(true)
    {
        botones();
        if(boton2==0) 
        {
            delay(20);
            digitalWrite(led1,HIGH);
            delay(100);
            digitalWrite(led1,LOW);
            delay(100);
            break;
        }
    }
    
    // Inicialización de la comunicación serial
    Serial.begin(115200);
}

void loop()
{
    // Llamar a la función de control PID
    pid(1,velocidad,KP,KI,KD);
    // Llamar a la función para detener el robot si se sale de la línea
    frenos_contorno(600);
    // Imprimir la posición de la línea
    Serial.println(position);
    delay(2);
}

void pid(int linea, int velocidad, float Kp, float Ki, float Kd)
{
    // Leer el valor de los sensores de línea
    position = qtra.readLine(sensorValues,QTR_EMITTERS_ON, linea, flanco_color, en_linea, ruido);
    
    // Calcular el error proporcional
    proporcional = (position) - 3500; // El punto de ajuste es 3500
    
    // Calcular la integral
    integral=integral + proporcional_pasado; 
    
    // Calcular el derivativo
    derivativo = (proporcional - proporcional_pasado);
    
    // Calcular la salida del control PID
    int ITerm=integral*KI;
    if(ITerm>=255) ITerm=255;
    if(ITerm<=-255) ITerm=-255;
    
    salida_pwm =( proporcional * KP ) + ( derivativo * KD )+(ITerm);
    
    // Limitar la salida de PWM
    if (  salida_pwm >velocidad )  salida_pwm = velocidad;
    if ( salida_pwm  <-velocidad )  salida_pwm = -velocidad;
    
    // Controlar los motores según la salida del PID
    if (salida_pwm < 0)
    {
        int der=velocidad-salida_pwm; 
        int izq=velocidad+salida_pwm;  
        if(der>=255)der=255;
        if(izq<=0)izq=0;
        motores(izq, der);
    }
    if (salida_pwm >0)
    {
        int der=velocidad-salida_pwm; 
        int izq=velocidad+salida_pwm; 
  
        if(izq >= 255) izq=255;
        if(der <= 0) der=0;
        motores(izq ,der );
    }

    // Actualizar el error proporcional pasado
    proporcional_pasado = proporcional;  
}

void frenos_contorno(int flanco_comparacion)
{
    // Detener el robot si se sale de la línea
    if (position <=10) // Si se sale por la parte derecha de la línea
    {
        while(true)
        { 
            digitalWrite(led1,HIGH);
            motores(-125,60);
            qtra.read(sensorValues); // Lectura en bruto de los sensores
            if ( sensorValues[0]<flanco_comparacion || sensorValues[1]<flanco_comparacion || sensorValues[2]<flanco_comparacion || sensorValues[3]<flanco_comparacion || sensorValues[4]<flanco_comparacion || sensorValues[5]<flanco_comparacion || sensorValues[6]<flanco_comparacion || sensorValues[7]<flanco_comparacion)
            {
                break;
            }
        }
    }

    if (position>=6990) // Si se sale por la parte izquierda de la línea
    {
        while(true)
        {
            digitalWrite(led1,HIGH);
            motores(60,-125); 
            qtra.read(sensorValues);
            if (sensorValues[7]<flanco_comparacion || sensorValues[6]<flanco_comparacion|| sensorValues[5]<flanco_comparacion || sensorValues[4]<flanco_comparacion || sensorValues[3]<flanco_comparacion || sensorValues[2]<flanco_comparacion || sensorValues[1]<flanco_comparacion|| sensorValues[0]<flanco_comparacion)
            {
                break;
            }
        }
    }
    digitalWrite(led1,LOW);
}

void motores(int motor_izq, int motor_der)
{
    // Control de los motores
    if ( motor_izq >= 0 )  
    {
        digitalWrite(mi1,LOW);
        digitalWrite(mi2,HIGH); 
        analogWrite(pwmi,motor_izq); 
    }
    else
    {
        digitalWrite(mi1,HIGH); 
        digitalWrite(mi2,LOW);
        motor_izq = motor_izq*(-1); 
        analogWrite(pwmi,motor_izq);
    }

    if ( motor_der >= 0 ) 
    {
        digitalWrite(md1,LOW);
        digitalWrite(md2,HIGH);
        analogWrite(pwmd,motor_der);
    }
    else
    {
        digitalWrite(md1,HIGH);
        digitalWrite(md2,LOW);
        motor_der= motor_der*(-1);
        analogWrite(pwmd,motor_der);
    }
}

void botones()
{
    // Leer los valores de los botones
    boton1=digitalRead(btn2);
    boton2=digitalRead(btn1); 
}
