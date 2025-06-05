#include <Arduino.h>
#include <Servo.h>

// === 핀 정의 ===
#define THROTTLE_IN A0 // 수신기에서 속도 PWM 입력
#define STEER_IN A1    // 수신기에서 조향 PWM 입력
#define THROTTLE_OUT 9 // ESC 출력
#define STEER_OUT 6    // 서보 출력

#define LED_RED 3
#define LED_GREEN 5

Servo esc;
Servo steer_servo;

void setup()
{
    esc.attach(THROTTLE_OUT);
    steer_servo.attach(STEER_OUT);

    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);

    esc.writeMicroseconds(1500); // ESC 중립
    steer_servo.write(90);       // 서보 중립 (90도)
    delay(2000);
}

void loop()
{
    // === 속도 제어 ===
    int pwm_throttle = pulseIn(THROTTLE_IN, HIGH, 25000);
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

    // === 조향 제어 ===
    int pwm_steer = pulseIn(STEER_IN, HIGH, 25000);  // 예: 1000~2000us
    int angle = map(pwm_steer, 1000, 2000, 40, 140); // 중립 90도
    angle = constrain(angle, 40, 140);
    steer_servo.write(angle);

    // === LED 제어: 후진 기준 ===
    if (pwm_throttle < 1450)
    {
        //int reverse_intensity = map(1500 - pwm_throttle, 0, 500, 0, 255); // 후진 속도 비례
        //analogWrite(LED_RED, reverse_intensity);
        //analogWrite(LED_GREEN, reverse_intensity);
        analogWrite(LED_RED, 1);
        analogWrite(LED_GREEN, 1);
    }
    else
    {
        // === LED 제어: 조향 기준 ===
        if (angle < 87)
        {
            // int brightness = map(90 - angle, 0, 50, 0, 255); // 좌회전 세기 → 밝기
            //  analogWrite(LED_RED, brightness);
            analogWrite(LED_RED, 1);
            analogWrite(LED_GREEN, 0);
        }
        else if (angle > 92)
        {
            //  int brightness = map(angle - 90, 0, 50, 0, 255); // 우회전 세기 → 밝기
            //   analogWrite(LED_GREEN, brightness);
            analogWrite(LED_GREEN, 1);
            analogWrite(LED_RED, 0);
        }
        else
        {
            analogWrite(LED_RED, 0);
            analogWrite(LED_GREEN, 0);
        }   
    }
}
