// Practice 2-2 
// 3차다항식으로 궤적 생성
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

void TIME_INTERRUPT_REGISTER()
{
  TCCR1A = 0;                               // TCCR - 제어 레지스터, PWM 모드 설정 및 분주비 설정 OC핀 출력을 설정하는 레지스터 
  TCCR1B = 0;
  TCCR1B |= (1 << WGM12);
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
  if (time < 5)
  {
    des = poly3(time, 0, 30.0, 5.0);
  }
}

//3차 다항식 궤적
double poly3 (double t, double init, double final, double tr)
{
  double tmp;
  a0 = init;
  a1 = 0;
  a2 = 3 * (final - init) / pow(tr,2);
  a3 = -2 * (final- init) / pow(tr,3);

  tmp = a0 + a1*t + a2*t*t + a3*t*t*t;
  return tmp;
}

//------------------------------------//
void setup() 
{
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
  Serial.begin(9600);   //시리얼통신 

  noInterrupts();
  TIME_INTERRUPT_REGISTER();
  interrupts();

  t_prev = micros();
}

// 시간을 요구하지 않는 간단한 동작 
void loop()   
{
  // put your main code here, to run repeatedly 
  // digitalWrite(13, LOW);
  // Serial.print(dt);
  // Serial.println("[us]");
  Serial.print(des);
  Serial.println("[degree]");  
}

