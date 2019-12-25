#include <Servo.h>

//---------------------------------------------------speed declaration-----------------------------------------------------------
int P, D, I;
float error, previousError;
int lsp, rsp;
int lfspeed = 130;//<<<<<<<<<<<<<<<<<---------Change speed here---------<<<<<<<<<<<<<<<<<<<<<<<
int minspeed = 0 ;
int maxspeed = 255;

long denominator = 0;
long numerator = 0;
float pos;
int numBlack = 0;

float Kp = 0.025;
float Kd = 1.48;
float Ki = 0.00023;
float PIDvalue = 0;
int lastposition = 3500;

Servo ESC;

int sensor[8][3];


void start()
{
  ESC.attach(9, 1000, 2000);
  digitalWrite(13, HIGH);
  
  ESC.write(0);
  delay(50);
  ESC.write(135);
  delay(1000);
  digitalWrite(13, LOW);
  while (!digitalRead(6)) {}
  motor1(100);
  motor2(100);
  delay(10);
}

//-------------------------------------------------------Calibrate----------------------------------------------------------------
void calibrate()
{
  for (int i = 0; i < 8; i++) // Initialising array values
  {
    sensor[i][0] = 500; //min values
    sensor[i][1] = 500; //max values
    sensor[i][2] = 500; //calibrated values//
  }
  int csp = 50;
  for (int a = 0; a < 4; a++)
  {
    motor1(csp);
    motor2(-csp);
    for (int c = 0; c < 1750; c++)
    {
      for (int i = 0; i < 8; i++)
      {
        if (analogRead(i) < sensor[i][0])
        {
          sensor[i][0] = analogRead(i);
        }
        if (analogRead(i) > sensor[i][1])
        {
          sensor[i][1] = analogRead(i);
        }
      }
    }
    csp = csp * -1;
  }
  digitalWrite(13, LOW);
  motor1(0);
  motor2(0);
}

void getLine()
{
  denominator = 0;
  numerator = 0;
  numBlack = 0;

  for (int i = 0; i < 8; i++)
  {
    sensor[i][2] = map(analogRead(i), sensor[i][0], sensor[i][1], 0, 1000);
    denominator += sensor[i][2];
    if (sensor[i][2] > 100) numBlack++;
  }

  numerator = ((sensor[1][2]) + (2 * sensor[2][2]) + (3 * sensor[3][2]) + (4 * sensor[4][2]) + (5 * sensor[5][2]) + (6 * sensor[6][2]) + (7 * sensor[7][2]));

  error = ((0.8 * sensor[3][2] + 2.4 * sensor[2][2] + 4 * sensor[1][2] + 8 * sensor[0][2])) - ((0.8 * sensor[4][2] + 2.4 * sensor[5][2] + 4 * sensor[6][2] + 8 * sensor[7][2]));
  if (numBlack > 0 && numBlack < 6)
  {
    pos = 1000 * numerator / denominator;
  }
}
//-------------------------------------------------------Setup Code----------------------------------------------------------------
void setup()
{
  Serial.begin(9600);
  ADCSRA &= ~(bit (ADPS0) | bit (ADPS1) | bit (ADPS2)); // clear prescaler bits

  // uncomment as required

  //  ADCSRA |= bit (ADPS0);                               //   2
  //  ADCSRA |= bit (ADPS1);                               //   4
  //ADCSRA |= bit (ADPS0) | bit (ADPS1);                 //   8
  ADCSRA |= bit (ADPS2);                               //  16
  //ADCSRA |= bit (ADPS0) | bit (ADPS2);                 //  32
  //  ADCSRA |= bit (ADPS1) | bit (ADPS2);                 //  64
  //  ADCSRA |= bit (ADPS0) | bit (ADPS1) | bit (ADPS2);   // 128


  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  calibrate();
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration
  start();
}

void loop()
{
  while (digitalRead(6))
  {
    getLine();
    if (pos < 1000)
    {
      lsp = -160;
      rsp = 150;
    }
    else if (pos > 6000)
    {
      lsp = 150;
      rsp = -160;
    }
    else
    {
      P = error;
      I = I + error;
      D = error - previousError;

      PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
      previousError = error;

      lsp = lfspeed - PIDvalue;
      rsp = lfspeed + PIDvalue;

      lsp = constrain(lsp, minspeed, maxspeed);
      rsp = constrain(rsp, minspeed, maxspeed);
    }

    motor1(rsp);
    motor2(lsp);
  }
  motor1(0);
  motor2(0);
  delay(50);
  motor1(-50);
  motor2(-50);
  ESC.write(0);
  delay(100);
  motor1(0);
  motor2(0);
  delay(1000);
}




void motor1(int spd)
{
  if (spd > 0)
  {
    digitalWrite(8, HIGH);
    digitalWrite(7, LOW);
    analogWrite(3, spd);
  }
  else if (spd < 0)
  {
    digitalWrite(8, LOW);
    digitalWrite(7, HIGH);
    analogWrite(3, spd * -1);
  }
  else
  {
    digitalWrite(8, LOW);
    digitalWrite(7, LOW);
    analogWrite(3, 0);

  }
}
void motor2(int spd)
{
  if (spd > 0)
  {
    digitalWrite(10, HIGH);
    digitalWrite(12, LOW);
    analogWrite(11, spd);
  }
  else if (spd < 0)
  {
    digitalWrite(10, LOW);
    digitalWrite(12, HIGH);
    analogWrite(11, spd * -1);
  }
  else
  {
    digitalWrite(10, LOW);
    digitalWrite(12, LOW);
    analogWrite(3, 0);
  }
}
