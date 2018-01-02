#include <mthread.h>
#include <DHT.h>

// LED pin used 22~24
#define RLED 23
#define GLED 22
#define BLED 24
#define FSTSONARTR 2 //First HCSR04's triger pin
#define FSTSONAREC 3 //First HCSR04's echo pin
#define SONARCNT 6  //Number of using sonar
#define DHTPIN 25   //AM2302 SDA pin
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
#define SUPPLY_VOLTAGE 1.1  //ADC 
//1번 이착륙장
#define ID 1
//지그비 메세지 송신 시 메세지의 마지막에 0x0D값을 넘겨줌
#define MSG_LAST 0x0D
//지그비 메세지 크기
#define ZB_BUFSIZ 4
//메세지 내 데이터 크기
#define DATSIZ 2
//위치 판단 범위
#define SRANGE 20.0
#define MRANGE 55.0
#define LRANGE 75.0

void LEDcontrol();
void printPos(uint16_t state);
void send2sys(uint16_t state);
uint8_t makeEqID(uint8_t id);
bool checkAck(uint16_t state);

DHT dht(DHTPIN, DHTTYPE);         //DHT센서제어 클래스
uint16_t pos_state = 0xFFFF;  //위치 상태 저장 변수

//위치 상태 변화시 작동할 이벤트핸들러 가상함수
class ChangedState_Event : public EventHandler
{
public:
  ChangedState_Event();
protected:
  bool condition();
private:
  uint16_t old_state;
};

ChangedState_Event::ChangedState_Event()
{
  old_state = 0;
}

//위치 상태 변화시 이벤트 발생
bool ChangedState_Event::condition()
{
  sleep_milli(1);
  if(old_state != pos_state)
  {
    old_state = pos_state;
    return true;
  }
  return false;
}

//상태변화시 씨리얼모니터와 LED에 출력하는 클래스
class Display_Event : public ChangedState_Event
{
protected:
  bool on_event();
};

bool Display_Event::on_event()
{
  LEDcontrol();
  printPos(pos_state);
  return false;
}

//상태변화시 지그비로 데이터를 전송하는 클래스
class Transport_Event : public ChangedState_Event
{
protected:
  bool on_event();
};

bool Transport_Event::on_event()
{
  send2sys(pos_state);
  sleep(1);
  if(checkAck(pos_state))  return false;
  return true;
}

//위치 상태 확인 쓰레드
class check_Thread : public Thread
{
protected:
  bool loop();
private: 
  unsigned long sonarAct(unsigned int trig, unsigned int echo);
  float pulse2cm(unsigned long microsec, float celsius);
  uint16_t checkPos(unsigned int fstTrig, unsigned int fstEcho, unsigned int count);
};

bool check_Thread::loop()
{
  // Die if requested:
  if(kill_flag)
      return false;
  pos_state = checkPos(FSTSONARTR, FSTSONAREC, SONARCNT);
  sleep_milli(1);
  return true;
}


unsigned long check_Thread::sonarAct(unsigned int trig, unsigned int echo) {
  digitalWrite(trig, LOW);
  sleep_micro(2);
  digitalWrite(trig, HIGH);
  sleep_micro(10);
  digitalWrite(trig, LOW);
  return pulseIn(echo, HIGH, 100000); //Limit to 100ms
}

//This is formula for calculate distance
float check_Thread::pulse2cm(unsigned long microsec, float celsius) {
  return (331.5+(0.6*celsius))*(float)microsec/20000.0;  
}

//위치 상태 점검 함수
//파라미터 (첫번째 트리거 핀번호, 첫번째 에코 핀번호, 초음파 갯수)
//반환값 (위치상태 데이터)
uint16_t check_Thread::checkPos(unsigned int fstTrig, unsigned int fstEcho, unsigned int count) {
  uint16_t bitmask, state;
  int i, j;
  float temperature, distance;
  state = 0;
  temperature = dht.readTemperature();
  //초음파 센서로 측정한 거리에 따라 위치 확인 후 해당 비트 체크. 위치별 비트는 개발문서 PPT 참조
  for(i=0, bitmask = 0x0004; i<count/2; i++, bitmask<<=3) {
    distance = pulse2cm(sonarAct(fstTrig+i*2, fstEcho+i*2), temperature);
    if(distance < SRANGE) state |= bitmask;
    else if(distance < MRANGE) state |= bitmask >> 1;
    else if(distance < LRANGE) state |= bitmask >> 2;
    sleep_milli(300);
  }
  for(i=count/2, bitmask = 0x0004; i<count; i++, bitmask>>=1) {
    distance = pulse2cm(sonarAct(fstTrig+i*2, fstEcho+i*2), temperature);
    if(distance < SRANGE) state |= bitmask;
    else if(distance < MRANGE) state |= bitmask << 3;
    else if(distance < LRANGE) state |= bitmask << 6;
    sleep_milli(300);
  }
  return state;
}

//LED상태 표시 함수
void LEDcontrol() {
  if(pos_state != 0)
  {
    if(pos_state & 0x0010)   //정중앙착륙
    {
      digitalWrite(RLED, HIGH);
      digitalWrite(GLED, LOW);
      digitalWrite(BLED, HIGH);
    }
    else  //외곽착륙
    {
      digitalWrite(RLED, LOW);
      digitalWrite(GLED, HIGH);
      digitalWrite(BLED, HIGH);
    }
  }
  else    //이륙
  {
    digitalWrite(RLED, HIGH);
    digitalWrite(GLED, HIGH);
    digitalWrite(BLED, LOW);
  }
}

//시리얼 모니터에 스위치 상태 표시 함수 (3x3환경에 최적화되어있음)
//파라미터 (스위치상태 데이터, 스위치 갯수)
void printPos(uint16_t state) {
  uint16_t bitmask = 0x0100;
  int i, j;

  Serial.println("OBJECT POSION");
  //물체의 위치를 3x3 공간에 표현. 해상도가 높아질 경우 직접 수정 요망
  for(i=0; i<SONARCNT/2; i++) {
    for(j=0; j<SONARCNT/2; j++) {
      if(state & bitmask) Serial.print("O ");
      else  Serial.print("X ");
      bitmask >>= 1;
    }
    Serial.println();
  }
  Serial.println();
}

//시스템으로 이착륙장 데이터 전송하는 함수 (4바이트의 데이터 송신)
//파라미터 (스위치상태 데이터)
void send2sys(uint16_t state) {
  int i;
  uint8_t buf[ZB_BUFSIZ] = {0x00,};
  buf[0] = makeEqID(1);
  for(i = 1; i <= DATSIZ; i++)  buf[i] = state >> 8 * (DATSIZ - i);  //2바이트의 데이터를 1바이트의 배열에 복사
  buf[ZB_BUFSIZ-1] = MSG_LAST;  //메시지의 마지막 바이트
  for(i = 0; i < ZB_BUFSIZ; i++)  Serial3.write(buf[i]);
}

//장비번호 생성함수
//파라미터 (고유번호)
//반환값 (장비고유번호 [장비식별번호+고유번호])
uint8_t makeEqID(uint8_t id) {
  uint8_t eqID = 0x40;  //최상위 2비트(이착륙장 식별비트 01, 게이트 식별비트 10)
  if(id < 1 || id > 0x3F)   return 0;   //장비번호는 01~3F(63)까지
  eqID |= id;
  return eqID;
}

//그라운드 시스템으로부터 ACK메세지 확인 함수
//ACK메세지 구조(장비고유번호 1byte+데이터 4byte+캐리지리턴 1byte)
bool checkAck(uint16_t state)
{
  int i;
  uint8_t buf[ZB_BUFSIZ] = {0x00,};
  uint16_t data = 0;
  if(Serial3.available() > 0)
  {
    Serial3.readBytes(buf, ZB_BUFSIZ);
    if((buf[0] & 0xC0) && (buf[ZB_BUFSIZ-1] == 0x0D))   //그라운드 시스템 식별용 최상위 2비트 (11)
    {
      for(i = 1; i <= DATSIZ; i++)
      {
        data |= buf[i];
        if(i != DATSIZ) data <<= 8;
      }
      if(data == state)
      {
        Serial3.flush();
        return true;
      }
    }
    return false;
  }
}

void setup() {
  int i;
  Serial.begin(9600);   //컴퓨터 시리얼 모니터
  Serial3.begin(9600);  //지그비
  dht.begin();
  analogReference(INTERNAL1V1); //This option accept for only Arduino mega
  //Ultrasonic sensor initialize
  for(i=0; i<SONARCNT; i++) {
    pinMode(FSTSONARTR+i*2, OUTPUT);
    pinMode(FSTSONAREC+i*2, INPUT);
  }
  //LED initalize
  pinMode(RLED, OUTPUT);
  pinMode(GLED, OUTPUT);
  pinMode(BLED, OUTPUT);
  //Add threads
  main_thread_list->add_thread(new Display_Event);
  main_thread_list->add_thread(new Transport_Event);
  main_thread_list->add_thread(new check_Thread);
  Serial.print("Landing Platform Operational");
  delay(1000);
}
