// Practice 7
// init pos -> x방향 이동 -> 복귀 
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

void Timer3_init()
{
  DDRB |= 0b01100000;
  TCCR3A = 0b10100010;
  TCCR3B = 0b00011010;
  ICR3 = 39999;
  OCR3A = 999; //20ms 주기
}

void Timer4_init()
{
  DDRB |= 0b01100000;
  TCCR4A = 0b10100010;
  TCCR4B = 0b00011010;
  ICR4 = 39999;
  OCR4A = 999; //20ms 주기
}

// 레지스터 설정 주기마다 타임인터럽트 실행 
ISR(TIMER1_COMPA_vect)    
{
  time += 0.001;


  t_now = micros();
  dt = (t_now - t_prev);
  t_prev = t_now;


  if (time < 3)
  {
    th1 = poly3(time, 0.0, 30.0, 3.0);
    th2 = poly3(time, 0.0, 120.0, 3.0);
  }

  else if (time < 6)
  {
    xd = poly3(time-3.0, 0.0, -50.0, 3.0);     //[mm]
    yd = poly3(time-3.0, 150.0, 150.0, 3.0);   // y 는 고정인데 그냥 150 상수로 넣어도 괜찮을까 ? 
    th2 = PI - acos((l1*l1 + l2*l2 - xd*xd - yd*yd)/(2*l1*l2));
    th1 = atan2(yd, xd) - atan2(l2*sin(th2), l1+l2*cos(th2));
    th2 = th2 *R2D;
    th1 = th1 *R2D;
  }
  else if (time < 9)
  {
    xd = poly3(time-6.0, -50.0, 0.0, 3.0);     //[mm]
    yd = poly3(time-6.0, 150.0, 150.0, 3.0);   
    th2 = PI - acos((l1*l1 + l2*l2 - xd*xd - yd*yd)/(2*l1*l2));
    th1 = atan2(yd, xd) - atan2(l2*sin(th2), l1+l2*cos(th2));
    th2 = th2 *R2D;
    th1 = th1 *R2D;   //deg 로 변환해줘야한다 ! 
  }
  servoOut1(th1);
  servoOut2(th2);
}

//3차 다항식 궤적
double poly3 (double t, double init, double final, double tr)
{
  double tmp=0.0;
  a0 = init;
  a1 = 0;
  a2 = 3 * (final - init) / pow(tr,2);
  a3 = -2 * (final- init) / pow(tr,3);

  tmp = a0 + a1*t + a2*t*t + a3*t*t*t;
  return tmp;
}



//서보모터 
void servoOut1(float deg)
{
  int duty; 

  //setup limitation 
  if (deg>180) deg=180;
  else if (deg<0) deg = 0;
  duty = (int)(999+(4999-999)/180*deg);   //999 -> 1 ms 이고, 4999 -> 2 ms 
  OCR3A = duty;
}

void servoOut2(float deg)
{
  int duty; 
  //setup limitation 
  if (deg>180) deg=180;
  else if (deg<0) deg = 0;
  duty = (int)(999+(4999-999)/180*deg);   //999 -> 1 ms 이고, 4999 -> 2 ms 
  OCR4A = duty;
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

  Timer3_init();
  Timer4_init();

  t_prev = micros();
}

// 시간을 요구하지 않는 간단한 동작 
void loop()   
{
  // Serial.print("time :");       Serial.println(time);
  // Serial.print("th1 :");        Serial.println(th1);
  // Serial.print("th2 :");        Serial.println(th2);
  // Serial.print("xd :");         Serial.println(xd);
  // Serial.print("yd :");         Serial.println(yd);
  // Serial.println("------------------------------");
}

