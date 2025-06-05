#include <Arduino.h>
#include <Servo.h>
#include <PinChangeInterrupt.h>

// === 핀 정의 ===
#define THROTTLE_IN A0
#define STEER_IN A1
#define THROTTLE_OUT 9
#define STEER_OUT 6
#define LED_RED 3
#define LED_GREEN 5
#define CH7_PIN 2 // 모드 전환용 PWM 핀

// === 객체 및 변수 ===
Servo esc;
Servo steer_servo;

volatile unsigned long ch7_start = 0;
volatile uint16_t ch7_pulse = 1500;
volatile bool new_ch7 = false;

bool mode = false; // false: 수동, true: 자동
String inputString = "";
bool stringComplete = false;

int last_angle = 90;

void ch7_ISR(); // CH7 PWM 측정 인터럽트 함수 선언

void setup()
{
    Serial.begin(9600);
    esc.attach(THROTTLE_OUT);
    steer_servo.attach(STEER_OUT);

    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(CH7_PIN, INPUT_PULLUP); // CH7 핀 입력 모드 설정

    esc.writeMicroseconds(1500);
    steer_servo.write(90);
    delay(2000);

    attachPCINT(digitalPinToPCINT(CH7_PIN), ch7_ISR, CHANGE);
    inputString.reserve(20);
}

void loop()
{
    // === 모드 갱신 ===
    if (new_ch7)
    {
        new_ch7 = false;
        mode = (ch7_pulse > 1500); // true: 자동, false: 수동
    }

    if (mode)
    {
        analogWrite(LED_RED, 0);
        analogWrite(LED_GREEN, 0);


        // === 자동 모드 ===
        while (Serial.available())
        {
            char inChar = (char)Serial.read();
            if (inChar == '\n')
            {
                stringComplete = true;
            }
            else
            {
                inputString += inChar;
            }
        }

        if (stringComplete)
        {
            int steer_val = inputString.toInt();
            steer_val = constrain(steer_val, -50, 50);
            int servo_pos = constrain(90 + steer_val, 40, 140);
            last_angle = servo_pos;
            steer_servo.write(servo_pos);
            esc.writeMicroseconds(1551); // 일정 전진

            inputString = "";
            stringComplete = false;
        }

        // === LED 제어: 조향 각도 기준 (자동 모드) ===
        // if (last_angle < 87)
        // {
        //     analogWrite(LED_RED, 1);
        //     analogWrite(LED_GREEN, 0);
        // }
        // else if (last_angle > 1)
        // {
        //     analogWrite(LED_GREEN, 1);
        //     analogWrite(LED_RED, 0);
        // }
        // else
        // {
        //     analogWrite(LED_RED, 0);
        //     analogWrite(LED_GREEN, 0);
        // }
    }
    else
    {
        // === 수동 모드 ===
        int pwm_throttle = pulseIn(THROTTLE_IN, HIGH, 25000);
        int pwm_steer = pulseIn(STEER_IN, HIGH, 25000);

        // 속도 제어
        int neutral = 1500;
        int reduced_pwm;

        if (pwm_throttle > neutral)
        {
            reduced_pwm = neutral + (pwm_throttle - neutral) / 5;
        }
        else if (pwm_throttle < neutral)
        {
            reduced_pwm = neutral - (neutral - pwm_throttle) / 5;
        }
        else
        {
            reduced_pwm = neutral;
        }
        esc.writeMicroseconds(reduced_pwm);

        // 조향 각도
        int angle = map(pwm_steer, 1000, 2000, 40, 140);
        angle = constrain(angle, 40, 140);
        steer_servo.write(angle);
        last_angle = angle;

        // LED 제어
        if (pwm_throttle < 1450)
        {
            analogWrite(LED_RED, 80);
            analogWrite(LED_GREEN, );
        }
        else
        {
            if (angle < 87)
            {
                analogWrite(LED_RED, 80);
                analogWrite(LED_GREEN, 0);
            }
            else if (angle > 92)
            {
                analogWrite(LED_GREEN, 80);
                analogWrite(LED_RED, 0);
            }
            else
            {
                analogWrite(LED_RED, 0);
                analogWrite(LED_GREEN, 0);
            }
        }
    }
}

// === CH7 PWM 측정 인터럽트 ===
void ch7_ISR()
{
    if (digitalRead(CH7_PIN) == HIGH)
        ch7_start = micros();
    else if (ch7_start)
    {
        ch7_pulse = micros() - ch7_start;
        new_ch7 = true;
        ch7_start = 0;
    }
}
