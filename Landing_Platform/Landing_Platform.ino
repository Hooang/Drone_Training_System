// pin used 22~30
#define FST_SW 22
#define SW_CNT 9
//1번 이착륙장
#define ID 1
//지그비 메세지 송신 시 메세지의 마지막에 0x0D값을 넘겨줌
#define MSG_LAST 0x0D

word switch_state;

void setup() {
  int i;
  for(i = 0 ; i < SW_CNT ; i++)  pinMode(FST_SW + i, INPUT_PULLUP);   //스위치
  Serial.begin(9600);   //컴퓨터 시리얼 모니터
  Serial3.begin(9600);  //지그비
  switch_state = 0;
}

void loop() {
  switch_state = checkSwitch(FST_SW, SW_CNT);
  printSwitch(switch_state, SW_CNT);  
  send2sys(switch_state);
  delay(1000);
}

//스위치 상태 점검 함수
//파라미터 (첫번째 스위치의 포트 번호, 스위치 갯수)
//반환값 (스위치상태 데이터)
word checkSwitch(int fSwitchNum, int count) {
  word bitmask, state;
  int i;
  state = 0;
  for(i = 0, bitmask = 0x0001; i < count; i++, bitmask <<= 1 )
    if(!digitalRead(fSwitchNum+i))   state|=bitmask;
  return state;
}

//시리얼 모니터에 스위치 상태 표시 함수 (3x3환경에 최적화되어있음)
//파라미터 (스위치상태 데이터, 스위치 갯수)
void printSwitch(word state, int count) {
  word bitmask;
  int i, j;

  for(i = 0, bitmask = 0x0001; i < (count / 3); i++)
  {
    for(j = 0; j < (count / 3); j++, bitmask <<= 1)
    {
      if(state & bitmask)  Serial.print("O ");
      else  Serial.print("X ");
    }
    Serial.println();
  }
  Serial.println();
}

//시스템으로 이착륙장 데이터 전송하는 함수 (4바이트의 데이터 송신)
//파라미터 (스위치상태 데이터)
void send2sys(word state) {
  int i;
  uint8_t buf[4] = {0x00,};
  buf[0] = makeEqID(1);
  for(i = 0; i < 2; i++)  buf[1+i] = state >> 8 * (1 - i);
  buf[3] = MSG_LAST;  //메시지의 마지막 바이트
  for(i = 0; i < 4; i++)  Serial3.write(buf[i]);
}

//장비번호 생성함수
//파라미터 (고유번호)
//반환값 (장비고유번호 [장비식별번호+고유번호])
uint8_t makeEqID(uint8_t id) {
  uint8_t eqID = 0x40;  //최상위 2비트(이착륙장 식별번호 01, 게이트 식별번호 10)
  if(id < 1 || id > 0x2F)   return 0;   //장비번호는 01~4F(63)까지
  eqID |= id;
  return eqID;
}

