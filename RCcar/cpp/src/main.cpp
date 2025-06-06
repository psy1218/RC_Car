#include <Arduino.h> //아두이노 기본 라이브러리
#include <Servo.h> //rc car의 속도와 조향을 제어하기 위한 서보 라이브러리
#include <PinChangeInterrupt.h> //인터럽트를 실시간으로 처리하기 위한 라이브러리

// === 핀 정의 ===
#define THROTTLE_IN A0      // 수신기에서 속도 PWM 입력
#define STEER_IN A1     // 수신기에서 조향 PWM 입력
#define THROTTLE_OUT 9   // ESC pwm 출력 핀
#define STEER_OUT 6     // 서보 pwm 출력 핀
#define LED_RED 3    //led red 핀
#define LED_blue 5   //led blue 핀
#define CH7_PIN 2        // 모드 전환용 PWM 핀
#define CH6_PIN 4        // 긴급사출 작동용 PWM 핀
#define AUX_SERVO_PIN 10 // 긴급사출 서보모터 핀

// 상태 저장 변수
bool isBackward = false;  // 후진 상태 여부
bool isTurningLeft = false; // 좌회전 상태 여부
bool isTurningRight = false; // 우회전 상태 여부

// === 객체 및 변수 ===
Servo esc; // ESC 제어용 서보
Servo steer_servo; // 조향 제어용 서보
Servo aux_servo; // 긴급사출 제어용 서보

// CH7 관련 - 모드 전환용: 수동/자동
volatile unsigned long ch7_start = 0; // CH7 PWM 측정용
volatile uint16_t ch7_pulse = 1500; // CH7 PWM 값 (중립 1500us)
volatile bool new_ch7 = false; // CH7 값이 갱신되었는지 여부
bool mode = false; // false: 수동, true: 자동

// CH6 관련 - 긴급사출용용
volatile unsigned long ch6_start = 0; // CH6 PWM 측정용
volatile uint16_t ch6_pulse = 1500; // CH6 PWM 값 (중립 1500us)
volatile bool new_ch6 = false; // CH6 값이 갱신되었는지 여부

// 시리얼 수신 처리용
String inputString = ""; // 시리얼 입력 문자열
bool stringComplete = false; // 문자열 수신 완료 여부

void ch7_ISR(); // CH7 PWM 측정 인터럽트 선언
void ch6_ISR(); // CH6 PWM 측정 인터럽트 선언

void setup(){
    Serial.begin(9600); // 시리얼 통신 초기화
    esc.attach(THROTTLE_OUT); // ESC는 핀 9번에 연결
    steer_servo.attach(STEER_OUT);  // 조향 서보는 핀 6번에 연결
    aux_servo.attach(AUX_SERVO_PIN); // 긴급사출 서보는 핀 10번에 연결

    pinMode(LED_RED, OUTPUT); // LED 빨간색 핀 출력 설정
    pinMode(LED_blue, OUTPUT); // LED 파란색 핀 출력 설정
    pinMode(CH7_PIN, INPUT_PULLUP); // CH7 모드 전환용 핀 입력 설정 (풀업 저항 사용)
    pinMode(CH6_PIN, INPUT_PULLUP); // CH6 긴급사출용 핀 입력 설정 (풀업 저항 사용)

    // === 초기화 ===  setting 단계에서 LED를 꺼져있도록 설정
    digitalWrite(LED_RED, LOW); // LED 빨간색 초기화
    digitalWrite(LED_blue, LOW); // LED 파란색 초기화

    esc.writeMicroseconds(1500); // ESC 초기값 중립
    steer_servo.write(90); // 서보 초기값 중립 (90도)
    aux_servo.write(0); // 긴급사출 서보 초기값 -> OFF 상태
    delay(2000); // 초기화 대기 시간

    //CHANGE 이용으로 falling, rising 전부 감지하여 pwm 측정
    attachPCINT(digitalPinToPCINT(CH7_PIN), ch7_ISR, CHANGE); // CH7 모드 전환용 PWM 측정 인터럽트 설정 
    attachPCINT(digitalPinToPCINT(CH6_PIN), ch6_ISR, CHANGE); // CH6 긴급사출용 PWM 측정 인터럽트 설정

    // === 시리얼 입력 초기화 ===
    inputString.reserve(20);
}

void loop(){
    // === CH7 모드 갱신 ===
    if (new_ch7){ //PCINT 인터럽트로 CH7 값이 갱신되었을 때
        new_ch7 = false; // 갱신된 상태를 false로 설정 - 다음 갱신을 위해
        mode = (ch7_pulse > 1500); // true: 자동모드(auto_mode), false: 수동모드(manual_mode)
    }

    // === CH6 서보 제어 ===
    if (new_ch6){ //PCINT 인터럽트로 CH6 값이 갱신되었을 때
        new_ch6 = false; // 다음 갱신을 위해 false로 설정
        if (ch6_pulse > 1500) {// CH6 핀의 펄스가 1500us보다 크면 긴급사출 서보를 ON 상태로 설정
            aux_servo.write(180); // ON 상태
        }
        else{
            aux_servo.write(0); // OFF 상태
        }
    }

    // === 자동 모드 === auto_mode
    if (mode){
        while (Serial.available()){ // 시리얼 입력이 있는 동안 - 라즈베리파이5에서 입력된 steer 값이 있는 동안
            // 시리얼 입력을 읽어 문자열로 저장
            char inChar = (char)Serial.read(); // 시리얼로부터 문자 단위로 읽기
            if (inChar == '\n') // 줄바꿈 문자가 들어오면
                stringComplete = true; // 문자열 수신 완료로 설정
            else
                inputString += inChar; // 입력된 문자를 문자열에 추가
        }
        //시리얼 입력값을 받고, 문자열과 줄바꿈 문자에 대한 처리 


        if (stringComplete) { // 문자열 수신이 완료되었을 때 = 줄바꿈 문자가 들어와서 한 문자열을 다 읽었을 때
            int steer_val = inputString.toInt(); // 문자열을 정수로 변환: 입력된 steer 값은 문자열이기 때문에 정수형으로 변환 시켜줘야함.
            steer_val = constrain(steer_val, -50, 50); // steer 값의 범위를 -50에서 50으로 제한
            // 범위를 제한하는 이유는, 라즈베리파이5에서 입력된 steer 값이 -50에서 50 사이로 제한되어 있기 때문이다.
            // 또한 이 범위를 벗어나는 값은 조향에 영향을 주지 않도록 하기 위함이다.
            int servo_pos = constrain(90 + steer_val, 40, 140); // 서보 위치 계산: 중립 90도에서 steer 값만큼 이동
            //최저, 최대 조향 값을 설정하지 않은 이유는 갑작스러운 조향을 방지하기 위함이다.
            steer_servo.write(servo_pos); // 계산된 steer 값을 서보에 작용하여 조향 제어가 가능하게 한다.

            if (servo_pos < 90 - 30 || servo_pos > 90 + 30){ // 조향이 좌우로 30도 이상 벗어났을 때
                // 조향이 좌회전, 우회전 상태일 때
                esc.writeMicroseconds(1553); // 평소 속도보다 약간 낮은 속도로 회전하여 안정적으로 회전할 수 있도록 하였다.
            }
            else{
                esc.writeMicroseconds(1555); // 조향이 중립 상태일 때 상대적으로 높은 속도로 직진할 수 있도록 하였다.
            }

            // 다음 문자열을 받기 위해 밑의 작업을 수행한다.
            inputString = ""; // 문자열 초기화
            stringComplete = false; // 문자열 수신 완료 상태 초기화

            // 조향과 후진에 따른 led 제어를 위한 작업
            int angle = servo_pos; // 서보 위치를 angle 변수에 저장

            if (angle > 93){ // 조향이 좌회전 상태일 때
                isTurningLeft = true; // 좌회전 상태로 설정
                isTurningRight = false; // 우회전 상태는 false로 설정
            }
            else if (angle < 83){ // 조향이 우회전 상태일 때
                isTurningRight = true; // 우회전 상태로 설정
                isTurningLeft = false; // 좌회전 상태는 false로 설정
            }
            else { // 조향이 중립 상태일 때
                isTurningLeft = false;
                isTurningRight = false;
            }

            // === LED 출력 (자동 모드 회전 방향 기준)
            if (isTurningLeft){  // 좌회전 상태일 때
                // LED 빨간색 켜고, 파란색 끔
                digitalWrite(LED_RED, 1);
                digitalWrite(LED_blue, 0);
            }
            else if (isTurningRight){ // 우회전 상태일 때
                // LED 빨간색 끄고, 파란색 켬
                digitalWrite(LED_RED, 0);
                digitalWrite(LED_blue, 1);
            }
            else { // 조향이 중립 상태일 때
                digitalWrite(LED_RED, 0);
                digitalWrite(LED_blue, 0);
            }
        }
    }
    else{ // === 수동 모드 === manual_mode
        // pulseIn 함수는 지정된 핀에서 HIGH 상태가 지속되는 시간을 마이크로초 단위로 반환
        // 25000us(25ms) 이상의 신호가 없으면 타임아웃으로 간주하여 0을 반환
        int pwm_throttle = pulseIn(THROTTLE_IN, HIGH, 25000); //pulseIn 함수를 사용하여 THROTTLE_IN 핀에서 PWM 신호를 읽어옴
        int pwm_steer = pulseIn(STEER_IN, HIGH, 25000); // pulseIn 함수를 사용하여 STEER_IN 핀에서 PWM 신호를 읽어옴

        int neutral = 1500; // 중립값 설정 (1500us)
        int reduced_pwm; // 속도를 줄이기 위한 변수


        // pwm 값을 제어하여 속도를 줄이는 이유는, 
        // 높은 pwm 값이 들어오면 차량이 급격하게 움직일 수 있기 때문에
        // 차량의 안전성을 위해 속도를 줄이는 것이다.
        if (pwm_throttle > neutral){ // 수신기에서 받은 속도 값이 중립값보다 크면
            reduced_pwm = neutral + (pwm_throttle - neutral) / 5; // 중립값에 비례하여 속도를 줄임
        }
        else if (pwm_throttle < neutral){ // 수신기에서 받은 속도 값이 중립값보다 작으면
        reduced_pwm = neutral - (neutral - pwm_throttle) / 5; // 중립값에 비례하여 속도를 줄임
        }
        else{
        reduced_pwm = neutral;  // 수신기에서 받은 속도 값이 중립값과 같으면 속도를 0으로 설정
        }

        // 수신기에서 받은 pwm 속도 값을 연산한 후 계산된 값을 esc에 전달
        esc.writeMicroseconds(reduced_pwm);


        // === 조향 제어 ===
        int angle = map(pwm_steer, 1000, 2000, 40, 140); // 수신기에서 받은 조향 PWM 값을 40도에서 140도로 매핑
        angle = constrain(angle, 40, 140); // 조향 각도를 40도에서 140도로 제한
        //constrain 함수는 주어진 값이 지정된 범위 내에 있도록 제한하는 함수
        steer_servo.write(angle); // 계산된 조향 각도를 서보에 전달하여 조향 제어

        // === LED 제어 ===
        // === isBackward 판별만 따로 처리 ===
        if (reduced_pwm < 1450){ // 수신기에서 받은 속도 값이 중립값보다 작으면 후진 상태로 판단
            // 후진 상태일 때는 LED를 켜고, 조향 방향에 관계 없이 후진 상태로 판단
            isBackward = true;
        }
        else if (reduced_pwm >= 1450){
            isBackward = false; // 수신기에서 받은 속도 값이 중립값보다 크거나 같으면 후진 상태가 아님
        }

        // === angle에 따라 회전 방향 판단 (정지 여부와 관계 없이) ===
        if (angle > 93){ // 조향 각도가 93도보다 크면 좌회전 상태로 판단
            isTurningLeft = true; // 좌회전 상태로 설정
            isTurningRight = false; // 우회전 상태는 false로 설정
        }
        else if (angle < 80){ // 조향 각도가 80도보다 작으면 우회전 상태로 판단
            isTurningRight = true; // 우회전 상태로 설정
            isTurningLeft = false; // 좌회전 상태는 false로 설정
        }
        else{ // 조향 각도가 80도와 93도 사이에 있으면 정지 상태로 판단
            isTurningLeft = false; 
            isTurningRight = false;
        }

        // === LED 출력 (정지 + 회전 방향 반영)
        if (isBackward) {   // 후진 상태일 때
            // LED 빨간색과 파란색을 켬
            digitalWrite(LED_RED, 1);
            digitalWrite(LED_blue, 1);
        }
        else if (isTurningLeft)   { // 좌회전 상태일 때
            // LED 빨간색 켜고, 파란색 끔
            digitalWrite(LED_RED, 1);
            digitalWrite(LED_blue, 0);
        }
        else if (isTurningRight){ // 우회전 상태일 때
            // LED 빨간색 끄고, 파란색 켬
            digitalWrite(LED_RED, 0);
            digitalWrite(LED_blue, 1);
        }
        else{ // 조향이 중립 상태일 때
            // LED 빨간색과 파란색을 끔
            digitalWrite(LED_RED, 0);
            digitalWrite(LED_blue, 0);
        }
    }
} //

// === CH7 PWM 측정 인터럽트 ===
void ch7_ISR(){
    if (digitalRead(CH7_PIN) == HIGH) // CH7 핀의 상태가 HIGH일 때
        ch7_start = micros(); // 시작 시간을 기록
    else if (ch7_start)    { // CH7 핀의 상태가 LOW로 바뀌었고, 시작 시간이 기록되어 있다면
        // CH7 핀의 상태가 LOW로 바뀌었을 때, 시작 시간과 현재 시간을 비교하여 펄스 길이를 계산
        ch7_pulse = micros() - ch7_start; // 펄스 길이 계산
        new_ch7 = true; // 새로운 CH7 값이 갱신되었음을 표시
        ch7_start = 0; // 시작 시간 초기화
    }
}

// 위의 CH7_ISR 함수와 동일한 구조로 CH6_ISR 함수가 이루어져있음.
// === CH6 PWM 측정 인터럽트 ===
void ch6_ISR(){
    if (digitalRead(CH6_PIN) == HIGH) 
        ch6_start = micros();
    else if (ch6_start){
        ch6_pulse = micros() - ch6_start;
        new_ch6 = true;
        ch6_start = 0;
    }
}
