//Auto-Calibrate Routine

int P[] ={A0,A1,A2,A3,A4,A5};
#define MR1 11
#define MR2 6
#define ML1 10
#define ML2 9
//#ifndef PID_H_
#define PID_H_

//Define parameter
#define epsilon 0.01
#define dt 0.001             //5ms loop time
#define MAX  255                   //For Current Saturation
#define MIN 50
#define Kp  10


#define Kd  0//0.02
#define Ki  00

int start,End,dif=0;
int state=0;
int prestate=0;
float integral=0;
 int Max[6],Min[6],avg[6],S[6],b[6],dv;
 

 
 //PID. Returns float
 float PIDcal(float setpoint,float actual_position)
{
	static float pre_error = 0;
	//static float integral = 0;
        if(prestate!=state) integral=0;
	float error;
	float derivative;
	float output;

	//Caculate P,I,D
	error = (setpoint - actual_position);
        Serial.print(" ERO :   ");
        Serial.print(Kp*error);
	//In case of error too small then stop intergration
	if(abs(error) > epsilon)
	{
		integral = (integral + error*dt);
       Serial.print(" INT :   ");
        Serial.print(Ki*integral);
	}
	derivative = (error - pre_error)/dt;
        Serial.print(" DER :   ");
        Serial.print(Kd*derivative);
       
        
        output = abs(Kp*error + Ki*integral + Kd*derivative);
        
        	//Saturation Filter
	if(output > MAX)
	{
		output = MAX;
	}
	else if(output < MIN)
	{
		output = MIN;
	}
        //Update error
        pre_error = error;
        Serial.print(" OUtput   ");
        Serial.println(output);
        prestate=state;
 return output;
}

void setup() {
 Serial.begin(9600);
  ///start = micros();
  pinMode(ML1, OUTPUT);
  pinMode(ML2, OUTPUT);
  pinMode(MR1, OUTPUT);
  pinMode(MR2, OUTPUT);
  pinMode(2, INPUT);
  stp();
  calibrate();
  Align();
 /// Serial.begin(9600);
  
}

void loop() {
 // int buttonstate = digitalRead(2);
 rd_Sensor();
 int mspeed;
  //if(buttonstate==HIGH)
  {
   
    
 ///  Serial.print(" ");  
   /// Serial.print("Time ");
   ///Serial.print(" :  ");
   ///Serial.print(dif);
   ///Serial.print("   ");
 ///  Serial.println(dv);
     
  }
  
  switch(dv)
  {
    case -1  : state =1;
                forward(PIDcal(0,-1));
                      break;
    case  0  : state=1;
                  forward(PIDcal(0,-1));
                      break;
    case  1  : state=1;
                forward(PIDcal(0,-1));
                 break;
                 
  case 10  :  state=2;
                  SRight(PIDcal(0,1.3)); //S
                    break;
      case 11  : state=2;
                      SRight(PIDcal(0,1.3));//S
                  break;      
      case -10  : state=3;
                  SLeft(PIDcal(0,-1.3));//S
                  break;      
    case -11  : state=3;
                  SLeft(PIDcal(0,-1.3)); //S
                      break;
    case 110 : state=6;
                  XHRight(PIDcal(0,1.6));
                 break;
    
      
    case -110 :state=7;
              XHLeft(PIDcal(0,-1.6));
                 break;
    case 100 : state=6;
                XHRight(PIDcal(0,1.6));
                 break;
    case 111 : state=4;
                HRight(PIDcal(0,1.6));
                 break;   
    case -100 : state=7;
                XHLeft(PIDcal(0,-1.6));
                 break; 
    case -111 : state=5;
                HLeft(PIDcal(0,-1.6)); 
                 break; 
                    
    
    /*
         
               
    case 9   : SRight(PIDcal(0,1.3));
               break;
             
                   

   
   case 99  :SRight(PIDcal(0,1.6));
                    break;
    case 90  :SRight(PIDcal(0,1.6));
                   break;
                   */
      case 101 : state=2;
                  HRight(PIDcal(0,1.3));
                    break;                
            
             //  case -99  :{SLeft(PIDcal(0,-1.6));
          //      break;}
  //  case -90  :{SLeft(PIDcal(0,-1.6));
   //             break;}
    case -101 : {state=2;
                HLeft(PIDcal(0,-1.3));
                break;}
   
   // case -9   : SLeft(PIDcal(0,-1.3));
   //             break;
              
  
    case 5 : stp();  
 default : stp();    
  }
  //End = micros();
  //dif = End - start;
  //stop();
  //buttonstate=0;
}

void rd_Sensor()
 {/// start = micros();
 for(int i =0;i<6;i++)
{
   S[i]=analogRead(P[i]);
   if(S[i]>=((Max[i]+Min[i])/2)) b[i]=1;
   else if(S[i]<((Max[i]+Min[i])/2)) b[i]=0;
}
  
  dv = (b[0]*100  + b[1]*10 + b[2]) - (b[3] + b[4]*10 + b[5]*100);
  if(((b[0]||b[1]||b[2]||b[3]||b[4]||b[5])==0)||((b[0]&&b[1]&&b[2]&&b[3]&&b[4]&&b[5])==1)) dv=5;
 /// End = micros();
 /// dif = End-start;
  }
 
 void forward(float spd)
 { //static int integral=0;
   //if(state!=prestate) stp();
   analogWrite(ML2,0);
   analogWrite(MR2,0);
   analogWrite(MR1,0+spd);
   analogWrite(ML1,0+spd);
   //Serial.print("  spd SLEFT    ");
   //prestate=state;
   //Serial.print(spd);
 }
 
void SRight(float spd)
 {
     analogWrite(ML2,0);
   analogWrite(MR2,0);
   analogWrite(MR1,0.5*spd);
   analogWrite(ML1,spd);
 }
 void HRight(float spd)
 {//reverse();
   //if(state!=prestate) stp();
   analogWrite(ML2,0);
   analogWrite(MR2,spd);
   analogWrite(MR1,0);
   analogWrite(ML1,spd);
   //Serial.print("  spd SLEFT    ");
  // prestate=state;
   //Serial.print(spd);
 }
 void SLeft(float spd)
 {
   analogWrite(ML2,0);
   analogWrite(MR2,0);
   analogWrite(MR1,spd);
   analogWrite(ML1,0.5*spd);
 }
 void HLeft(float spd)
 {//stp();
 //reverse();
   //if(state!=prestate) stp();
   analogWrite(ML2,spd);
   analogWrite(MR2,0);
   analogWrite(MR1,spd);
   analogWrite(ML1,0);
   //Serial.print("  spd SLEFT    ");
   //prestate=state;
   //Serial.print(spd);
 }
 void stp()
 {
   analogWrite(ML2,255);
   analogWrite(MR2,255);
   analogWrite(MR1,255);
   analogWrite(ML1,255);
 }
 
 void XHRight(float spd)
 {  
   if(state!=prestate) 
   {
     reverse();
  //stp();
   }
   //delay(10);
   int flag2 =0 ,count = 0;
   while(flag2!=1 && count <30)
   { count++;
  // Serial.print("   countRight :    ");
   //Serial.println(count);
   rd_Sensor();   
   if(dv==111 || dv ==11)
   { //forward(30);
     flag2 =1;
     break;
   }
   else if(flag2==0)
   {
     rd_Sensor();   
   analogWrite(ML2,0);
   analogWrite(MR2,spd);
   analogWrite(MR1,0);
   analogWrite(ML1,spd);
   }
   }
   prestate=state;
 }
  void XHLeft(float spd)
 { if(state!=prestate) 
 {reverse();
  //stp();
 }
 ///delay(10);
   int flag1 = 0, count =0;
   while(flag1!=1 && count < 30)
   { count++;
  //  Serial.print("   countLeft :    ");
   //Serial.println(count);
     rd_Sensor();   
   if(dv == -111 || dv==-11)
   { //forward(30);
     flag1 =1;
     break;
   }
   else if(flag1==0)
   {
     rd_Sensor();
   analogWrite(ML2,spd);
   analogWrite(MR2,0);
   analogWrite(MR1,spd);
   analogWrite(ML1,0);
   }
   }
   prestate=state;
 }
 
 void reverse()
 {
   analogWrite(ML2,155);
   analogWrite(MR2,155);
   analogWrite(MR1,0);
   analogWrite(ML1,0);
  // delay(1);
 }
 
 
 void calibrate()
 {
  
   for(int i=0;i<6;i++) 
   {
     Max[i] = Min[i] = analogRead(P[i]);
   }
   int m1,m2;
   for(int g=0;g<2;g++)
   for(int k=0;k<2;k++)
   {  
     if(k==0)
     {
       m1=70;
       m2=0;
     }
     else if(k==1)
     {
       m1=0;
       m2=70;
     }
     for(int j=0;j<10;j++)
     {
       for(int L=0;L<6;L++)
       {
         S[L]=analogRead(P[L]);
     analogWrite(MR1,m1);
     analogWrite(MR2,m2);
     analogWrite(ML1,m2);
     analogWrite(ML2,m1);
     if(S[L]>Max[L]) Max[L]=S[L];
     if(S[L]<Min[L]) Min[L]=S[L];
    
      Serial.print(S[0]);
  Serial.print("   ");
    Serial.print(S[1]);
  Serial.print("   ");

  Serial.print(S[2]);
  Serial.print("   ");

  Serial.print(S[3]);
  Serial.print("   ");

  Serial.print(S[4]);
  Serial.print("   ");

  Serial.print(S[5]);
  Serial.println("   ");
  
     }
     stp();
     }
   }
   }
   
   void Align()
   {
     Serial.print("Aligning");
     int flag3=0;
   while(flag3!=1)
   {
   rd_Sensor();   
     if(dv==10||dv==11) 
     {
     flag3=1;
     //Serial.println("Exit While Loop");
     break;
     }
     else
     {
       //Serial.println("runnning");
       analogWrite(MR1,40);
     analogWrite(MR2,0);
     analogWrite(ML1,0);
     analogWrite(ML2,40);
     }
   }
  stp();
   }
 
   
 void serialprint()
 {
   Serial.println();
   for(int i=0;i<6;i++)
  {
    Serial.print("AVG");
    Serial.print(i);
    Serial.print("   ");
    Serial.print((Max[i]+Min[i])/2);
    Serial.print("   ");
    Serial.print("Value");
    Serial.print(i);
    Serial.print("   ");
  Serial.print(S[i]);
  Serial.print("   ");
  }
    delay(300);
  Serial.println(" ");
   
 }
 

