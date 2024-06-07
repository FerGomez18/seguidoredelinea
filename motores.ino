#define ain1        3
#define ain2        4
#define pwma        6
#define bin1        8
#define bin2        7
#define pwmb        5

void setup() 
{
  pinMode(ain1,OUTPUT);
  pinMode(ain2,OUTPUT);
  pinMode(bin1,OUTPUT);
  pinMode(bin2,OUTPUT);
}

void loop()
{
  motores(-500,500);
}

void motores(int motor_izq, int motor_der)
{
  if( motor_izq >=0)
    {
      digitalWrite (ain1,HIGH);
      digitalWrite (ain2,LOW);
      analogWrite (pwma,motor_izq);
    }
  else
    {
      digitalWrite(ain1,LOW);
      digitalWrite(ain2,HIGH);
      motor_izq = motor_izq*(-1);
      analogWrite(pwma,motor_izq); 
    }
  if(motor_der>=0)
    {
      digitalWrite(bin1,HIGH);
      digitalWrite(bin2,LOW);
      analogWrite(pwmb,motor_der);
  }
  else
     {
      digitalWrite(bin1,LOW);
      digitalWrite(bin2,HIGH);
      motor_der=motor_der*(-1);
      analogWrite(pwmb,motor_der);    
     }
}