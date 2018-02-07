#define MR1 11
#define MR2 6
#define ML1 10
#define ML2 9


//Function to setup correct pins to the motor and test its working

void setup()
{
  pinMode(ML1, OUTPUT);
  pinMode(ML2, OUTPUT);
  pinMode(MR1, OUTPUT);
  pinMode(MR2, OUTPUT);
}

void loop()
{
  //Right Motor Forward
  analogWrite(MR1,60);
  analogWrite(MR2,0);
  delay(1000);
  
  analogWrite(MR1,0);
  analogWrite(MR2,0);
  analogWrite(ML1,0);
  analogWrite(ML2,0);
  
  //Left Motor Forward 
  analogWrite(ML1,60);
  analogWrite(ML2,0);
  delay(1000);
    analogWrite(MR1,0);
  analogWrite(MR2,0);
  analogWrite(ML1,0);
  analogWrite(ML2,0);
  //Right Motor Forward
  analogWrite(MR2,60);
  analogWrite(MR1,0);
  delay(1000);
    analogWrite(MR1,0);
  analogWrite(MR2,0);
  analogWrite(ML1,0);
  analogWrite(ML2,0);
  //Left Motor Forward 
  analogWrite(ML2,60);
  analogWrite(ML1,0);
  delay(1000);
    analogWrite(MR1,0);
  analogWrite(MR2,0);
  analogWrite(ML1,0);
  analogWrite(ML2,0);
    analogWrite(MR1,60);
  analogWrite(MR2,0);
  analogWrite(ML1,60);
  analogWrite(ML2,0);
  delay(1000);
    analogWrite(MR1,0);
  analogWrite(MR2,0);
  analogWrite(ML1,0);
  analogWrite(ML2,0);
  
    analogWrite(MR1,0);
  analogWrite(MR2,60);
  analogWrite(ML1,0);
  analogWrite(ML2,60);
  delay(1000);
    analogWrite(MR1,0);
  analogWrite(MR2,0);
  analogWrite(ML1,0);
  analogWrite(ML2,0);
  
}
   
