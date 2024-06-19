// Practice 5
// 시리얼 모니터로 x,y 입력받고 역기구학 식 풀어서 th1, th2 출력하기 
// 20191091 김태민 

//-------------------------------------//
int led = 13; 
double t_prev;
double t_now;
double dt;
int CNT = 0;

double a0, a1, a2, a3; 
double time = 0.0;
double des = 0.0;
int target = 0;
int theta_0 = 0; 

double l1 = 150.0;
double l2 = 150.0;    //[mm]
double th1, th2;
double xd, yd;


#define R2D 180/PI



void TIME_INTERRUPT_REGISTER()
{
  TCCR1A = 0;                               // TCCR - 제어 레지스터, PWM 모드 설정 및 분주비 설정 OC핀 출력을 설정하는 레지스터 
  TCCR1B = 0;
  TCCR1B |= (1 << WGM12);                   //제어 레지스터, 
  TCCR1B |= (0<<CS12)|(0<<CS11)|(1<<CS10);
  TCNT1 = 0;
  OCR1A = 16000-1;                        // 1 ms  
  TIMSK1 |= (1<<OCIE1A);
}


// 레지스터 설정 주기마다 타임인터럽트 실행 
ISR(TIMER1_COMPA_vect)    
{
  time += 0.001;
  CNT++;

  t_now = micros();
  dt = (t_now - t_prev);
  t_prev = t_now;

  if (CNT % 1000 < 500)
  {
    digitalWrite(led, HIGH);
  }
  else 
  {
    digitalWrite(led, LOW);
  }
}


//------------------------------------//
void setup() 
{
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
  pinMode(5, OUTPUT);   //timer3
  pinMode(6, OUTPUT);   //timer4
  Serial.begin(9600);   //시리얼통신 

  noInterrupts();
  TIME_INTERRUPT_REGISTER();
  interrupts();

  // Timer3_init();
  // Timer4_init();

  // t_prev = micros();
}

// 시간을 요구하지 않는 간단한 동작 
void loop()   
{

  if(Serial.available()>0)
  {
    while (Serial.available() < 2 * sizeof(int))
    {
      // wait for enough data to be available
    }
    xd = Serial.parseInt();   //[mm]
    yd = Serial.parseInt();

    Serial.print("x:");
    Serial.print(xd);
    Serial.print("y:");
    Serial.print(yd);
    Serial.println();

    if (sqrt(xd*xd + yd*yd) > l1 + l2)
    {
      Serial.println("Impossible");
    }
    else 
    {
      th2 = PI - acos((l1*l1 + l2*l2 - xd*xd - yd*yd)/(2*l1*l2));
      th1 = atan2(yd, xd) - atan2(l2*sin(th2), l1+l2*cos(th2));

      Serial.print("th1: ");
      Serial.println(th1 * R2D); // 라디안에서 각도로 변환
      Serial.print(" th2: ");
      Serial.print(th2 * R2D); // 라디안에서 각도로 변환
      Serial.println();
    }

  }

  // put your main code here, to run repeatedly 
  // digitalWrite(13, LOW);
  // Serial.print(dt);
  // Serial.println("[us]");
  // Serial.print(des);
  // Serial.println("[degree]");  
}

