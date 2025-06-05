#include <Arduino.h>
#include <Servo.h>

#define STEER_PIN 6       // 조향 서보 핀
#define THROTTLE_PIN 9    // 속도 제어용 ESC 핀

Servo steer_servo;
Servo esc;

String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(9600);
  steer_servo.attach(STEER_PIN);
  esc.attach(THROTTLE_PIN);

  esc.writeMicroseconds(1500);  // 중립 신호
  delay(2000);

  inputString.reserve(10);  // 시리얼 입력 버퍼 크기
}
int flag = 0;
void loop() {

    while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }

  if (stringComplete) {
    //Serial.println("수신한 문자열: " + inputString);  // 🔍 확인용 출력

    int steer_val = inputString.toInt();

    // steering 값: -50 ~ +50
    // 서보는 90도를 중심으로 좌우로 조향
    steer_val = constrain(steer_val, -50, 50);
    int servo_pos = 90 + steer_val;  // 90이 중앙
    servo_pos = constrain(servo_pos, 40, 140);  // 서보 안전 범위
    steer_servo.write(servo_pos);

    // 속도 고정값 예시 (정지 상태 아님)
    if (servo_pos  < 90-30 || servo_pos > 90+30) {
      if(flag == 0) {
        esc.writeMicroseconds(1500); // 정지 상태
        flag =1;      
      }
      else{
        esc.writeMicroseconds(1551); // 전진 빠르게
   
      }
        
    } else {
      esc.writeMicroseconds(1553); // 정지 상태
      flag = 0;
    }




    // esc.writeMicroseconds(1553);// 전진 느리게

    inputString = "";
    stringComplete = false;
  }
}

// void serialEvent() {
//   while (Serial.available()) {
//     char inChar = (char)Serial.read();
//     if (inChar == '\n') {
//       stringComplete = true;
//     } else {
//       inputString += inChar;
//     }
//   }
// }
