#include <mthread.h>

// LED pin used 2~4
#define RLED 2
#define GLED 3
#define BLED 4
// SW pin used 22~42
#define FST_SW 22
#define SW_CNT 21
//1번 이착륙장
#define ID 1
//지그비 메세지 송신 시 메세지의 마지막에 0x0D값을 넘겨줌
#define MSG_LAST 0x0D
//지그비 메세지 크기
#define ZB_BUFSIZ 6
//메세지 내 데이터 크기
#define DATSIZ 4

void LEDcontrol();
uint32_t checkSwitch(int fSwitchNum, int count);
void printSwitch(uint32_t state);
void send2sys(uint32_t state);
uint8_t makeEqID(uint8_t id);
bool checkAck(uint32_t state);

uint32_t switch_state = 0xFFFFFFFF;  //스위치 상태 저장

//스위치 상태 변화시 작동할 이벤트핸들러 가상함수
class ChangedState_Event : public EventHandler
{
public:
  ChangedState_Event();
protected:
  bool condition();
private:
  uint32_t old_state;
};

ChangedState_Event::ChangedState_Event()
{
  old_state = 0;
}

//스위치의 상태 변화시 이벤트 발생
bool ChangedState_Event::condition()
{
  sleep_milli(1);
  if(old_state != switch_state)
  {
    old_state = switch_state;
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
  printSwitch(switch_state);
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
  send2sys(switch_state);
  sleep(1);
  if(checkAck(switch_state))  return false;
  return true;
}

//스위치 상태 확인 쓰레드
class check_Thread : public Thread
{
protected:
  bool loop();
};

bool check_Thread::loop()
{
  // Die if requested:
  if(kill_flag)
      return false;
  switch_state = checkSwitch(FST_SW, SW_CNT);
  sleep_milli(1);
  return true;
}

//LED상태 표시 함수
void LEDcontrol() {
  if(switch_state != 0)
  {
    if(switch_state & 0x00000400)   //정중앙착륙
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

//스위치 상태 점검 함수
//파라미터 (첫번째 스위치의 포트 번호, 스위치 갯수)
//반환값 (스위치상태 데이터)
uint32_t checkSwitch(int fSwitchNum, int count) {
  uint32_t bitmask, state;
  int i;
  state = 0;
  for(i = 0, bitmask = 0x00000001; i < count; i++, bitmask <<= 1 )
    if(!digitalRead(fSwitchNum+i))   state|=bitmask;
  return state;
}

//시리얼 모니터에 스위치 상태 표시 함수 (3x3환경에 최적화되어있음)
//파라미터 (스위치상태 데이터, 스위치 갯수)
void printSwitch(uint32_t state) {
  uint32_t bitmask = 0x00000001;
  int i, j;

  //1-1 ~ 1-3
  Serial.print("  ");
  for(i = 0; i < 3; i++)
  {
    if(state & bitmask)  Serial.print("O ");
    else  Serial.print("X ");   
    bitmask <<= 1;
  }
  Serial.println();

  //2-1 ~ 4-5
  for(i = 0; i < 3; i++)
  {
    for(j = 0; j < 5; j++)
    {
      if(state & bitmask)  Serial.print("O ");
      else  Serial.print("X ");   
      bitmask <<= 1;
    }
    Serial.println();
  }

  //5-1 ~ 5-3
  Serial.print("  ");
  for(i = 0; i < 3; i++)
  {
    if(state & bitmask)  Serial.print("O ");
    else  Serial.print("X ");   
    bitmask <<= 1;
  }
  Serial.println();
  Serial.println();
}

//시스템으로 이착륙장 데이터 전송하는 함수 (4바이트의 데이터 송신)
//파라미터 (스위치상태 데이터)
void send2sys(uint32_t state) {
  int i;
  uint8_t buf[ZB_BUFSIZ] = {0x00,};
  buf[0] = makeEqID(1);
  for(i = 1; i <= DATSIZ; i++)  buf[i] = state >> 8 * (DATSIZ - i);  //4바이트의 데이터를 1바이트의 배열에 복사
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
bool checkAck(uint32_t state)
{
  int i;
  uint8_t buf[ZB_BUFSIZ] = {0x00,};
  uint32_t data = 0;
  if(Serial3.available() > 0)
  {
    Serial3.readBytes(buf, ZB_BUFSIZ);
    if((buf[0] & 0xC0) && (buf[5] == 0x0D))   //그라운드 시스템 식별용 최상위 2비트 (11)
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
  pinMode(RLED, OUTPUT);
  pinMode(GLED, OUTPUT);
  pinMode(BLED, OUTPUT);
  for(i = 0 ; i < SW_CNT ; i++)   pinMode(FST_SW + i, INPUT_PULLUP);   //스위치
  main_thread_list->add_thread(new Display_Event);
  main_thread_list->add_thread(new Transport_Event);
  main_thread_list->add_thread(new check_Thread);
  delay(1000);
}
