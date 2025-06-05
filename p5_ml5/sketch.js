// ML5.js 라이브러리를 사용한 손동작 인식
let handPose;
let video;
let hands = [];
let port;

// 신호등 상태 및 밝기 변수
let redState = 0,
  yellowState = 0,
  greenState = 0,
  brightness = 0,
  mode = 0;

// 슬라이더 & 버튼
let redSlider, yellowSlider, greenSlider;
let connectBtn;
let modeButtons = [];

// 질문 & 선택 관련 변수
let currentStep = 0;
let selectedFunction = null;
let selectedButton = null;
let selectedColor = null;

// 버튼 상태 변수 추가 (홀/짝 토글)
let button1flag = 0;
let button2flag = 0;
let button3flag = 0;

// 이전 프레임에서의 제스처 상태 (각 질문별로 따로 관리)
let prevPalm_0 = false,
  prevVSign_0 = false; // 첫 번째 질문 (버튼/슬라이더 선택)
let prevPalm_1 = false,
  prevVSign_1 = false,
  prevOk_1 = false; // 버튼 선택
let prevPalm_2 = false,
  prevVSign_2 = false,
  prevOk_2 = false; // 색상 선택
let prevPointingUp_3 = false,
  prevPointingDown_3 = false; // 신호 조정

// 신호 지속 시간 조정 관련 변수
let lastSentTime = 0;
let sendInterval = 3000; // 100ms 주기

let currentGesture = ""; // 현재 인식된 손동작 (예: "보자기", "브이" 등)
//------------------------------------------------------------------------------------

//ml5.js 손동작 인식 모델 로드
function preload() {
  handPose = ml5.handPose();
}

function setup() {
  createCanvas(1280, 480); // 2개의 화면을 나란히 표시 - 비디오와 UI 
  video = createCapture(VIDEO, { flipped: true });
  video.size(640, 480); 
  video.hide();
  handPose.detectStart(video, gotHands);

  // 시리얼 포트 생성 및 연결
  port = createSerial();
  if (usedSerialPorts().length > 0) {
    port.open(usedSerialPorts()[0], 9600);
  }

  // Web Serial 연결 버튼
  connectBtn = createButton("Connect to Arduino");
  connectBtn.position(900, 10);
  connectBtn.mousePressed(connectBtnClick);

  // 모드 변경 버튼
  let modeLabels = ["Normal", "Button1", "Button2", "Button3"];
  for (let i = 0; i < 4; i++) {
    modeButtons[i] = createButton(modeLabels[i]);
    modeButtons[i].position(730 + i * 100, 440);
    modeButtons[i].mousePressed(() => toggleMode(i));
  }

  // 슬라이더 생성 (LED 지속 시간 조절)
  redSlider = createSlider(0, 4000, 2000);
  redSlider.position(740, 330);
  redSlider.input(sendData);

  yellowSlider = createSlider(0, 4000, 500);
  yellowSlider.position(895, 330);
  yellowSlider.input(sendData);

  greenSlider = createSlider(0, 4000, 2000);
  greenSlider.position(1050, 330);
  greenSlider.input(sendData);
}

function draw() {
  background(30);
  image(video, 0, 0, width - 640, height);

  // 캔버스 오른쪽 영역 (2번 화면처럼 사용)
  fill(250, 200, 250);
  rect(640, 0, 640, 480);

  drawHandJoints();

  // 시리얼 데이터 수신 (draw() 내부에서 처리)
  if (port.available()) {
    // 데이터 수신 시
    let data = port.readUntil("\n"); // 한 줄 단위로 읽기
    processSerialData(data.trim()); // 개행문자 제거 후 처리
  }

  // 신호 길이 슬라이더 및 값 표시
  textSize(14);
  // 텍스트 표시
  fill(255, 255, 255);
  text("0s", 740, 320);
  text("4s", 865, 320);
  text("0s", 895, 320);
  text("4s", 1025, 320);
  text("0s", 1050, 320);
  text("4s", 1175, 320);

  // 슬라이더 값 주기적으로 전송
  if (millis() - lastSentTime > sendInterval) {
    sendData();
    lastSentTime = millis();
  }

  //기능 draw 함수
  detectGesture();
  displayUI();
  drawTrafficLights();
  drawBrightnessGauge();
  drawModeIndicator();
}

// 손동작 인식 및 제스처 처리
function detectGesture() {
  noStroke();

  if (hands.length > 0) {
    let hand = hands[0];
    let landmarks = hand.keypoints;

    //제스처 상태 저장 변수
    let ok = isOKSign(landmarks);
    let vSign = isVSign(landmarks);
    let palm = isPalm(landmarks);
    let pointingUp = isPointingUp(landmarks);
    let pointingDown = isPointingDown(landmarks);
    let rockAndRoll = isRockAndRoll(landmarks); // 뒤로 가기

    // 현재 인식된 손동작 표시 및 저장 
    if (palm) {
      currentGesture = "🖐️ 보자기";
    } else if (vSign) {
      currentGesture = "✌️ 브이";
    } else if (ok) {
      currentGesture = "👌 오케이";
    } else if (pointingUp) {
      currentGesture = "☝️ 위로";
    } else if (pointingDown) {
      currentGesture = "👇 아래로";
    } else if (rockAndRoll) {
      currentGesture = "🤘 락앤롤";
    } else {
      currentGesture = ""; // 인식된 제스처 없음
    }

    //currentstep을 활용하여 질문의 답 저장 및 질문 이동

    // ✅ 락앤롤 제스처로 뒤로 가기 기능 실행 (첫 질문 제외)
    if (rockAndRoll && !prevRockAndRoll) {
      if (currentStep === 2) {
        currentStep = 0; // 색상 선택 단계에서는 첫 질문으로 돌아감
      } else if (currentStep > 0) {
        currentStep--; // 일반적인 경우 한 단계 뒤로 가기
      }
    }

    if (currentStep === 0) {
      // ✅ ① 기능 선택
      if (palm && !prevPalm_0) {
        selectedFunction = "button";
        currentStep = 1;
      } else if (vSign && !prevVSign_0) {
        selectedFunction = "slider";
        currentStep = 2;
      }
    } else if (currentStep === 1) {
      // ✅ ②-1 버튼 선택 (prevPalm_1, prevVSign_1, prevOk_1 사용)
      if (palm && !prevPalm_1) {
        button1flag++;
        toggleButton("button1", button1flag);
      } else if (vSign && !prevVSign_1) {
        button2flag++;
        toggleButton("button2", button2flag);
      } else if (ok && !prevOk_1) {
        button3flag++;
        toggleButton("button3", button3flag);
      }
    } else if (currentStep === 2) {
      // ✅ ②-2 색상 선택 (prevPalm_2, prevVSign_2, prevOk_2 사용)
      if (palm && !prevPalm_2) {
        selectedColor = "red";
        currentStep = 3;
      } else if (vSign && !prevVSign_2) {
        selectedColor = "yellow";
        currentStep = 3;
      } else if (ok && !prevOk_2) {
        selectedColor = "green";
        currentStep = 3;
      }
    } else if (currentStep === 3) {
      // ✅ ③ 신호 지속 시간 조정 (prevPointingUp_3, prevPointingDown_3 사용)
      //if (pointingUp && !prevPointingUp_3) {
      if (pointingUp) {
        print("pointingup");
        adjustSlider(selectedColor, 30);
        // } else if (pointingDown && !prevPointingDown_3) {
      } else if (pointingDown) {
        adjustSlider(selectedColor, -30);
      }
    }

    // ✅ 이전 프레임 상태 업데이트
    // 여러번 인식되는 것을 방지하기 위해 이전 프레임 상태를 저장
    prevPalm_0 = palm;
    prevVSign_0 = vSign;
    prevPalm_1 = palm;
    prevVSign_1 = vSign;
    prevOk_1 = ok;
    prevPalm_2 = palm;
    prevVSign_2 = vSign;이진
    prevOk_2 = ok;
    // prevPointingUp_3 = pointingUp; prevPointingDown_3 = pointingDown;
    prevRockAndRoll = rockAndRoll;
  }
}

// UI 표시
function displayUI() {
  noStroke();
  fill(0, 0, 0);
  textSize(16);
  textAlign(LEFT, CENTER); // 왼쪽 정렬 적용

  let startX = 700; // 왼쪽 여백
  let startY = 60; // 첫 번째 줄 시작 위치
  let lineHeight = 30; // 줄 간격

  // ✅ 현재 손동작 인식 결과 표시
  if (currentGesture) {
    let gestureText = "🙌 현재 인식된 손동작: " + currentGesture;

    // 배경 박스 먼저 그림
    let padding = 5;
    let boxX = startX + 300 - padding;
    let boxY = height - 340 - 12; // textSize 16 기준 높이 맞춤
    let boxW = textWidth(gestureText) + padding * 2;
    let boxH = 24;

    fill(255, 230, 180); // 연한 주황 배경
    noStroke();
    rect(boxX, boxY, boxW, boxH, 6);

    // 텍스트 위에 출력
    fill(0);
    textAlign(LEFT, CENTER);
    text(gestureText, startX + 300, height - 340);
  }

  // ✅ 현재까지 선택한 값 표시
  if (currentStep > 0) {
    let selectedText = "🛠️ 선택한 기능: ";
    if (selectedFunction === "button") {
      selectedText += "🔘 버튼 모드";
      if (selectedButton) selectedText += ` | 버튼: ${selectedButton}`;
    } else if (selectedFunction === "slider") {
      selectedText += "🎨 슬라이더 모드";
      if (selectedColor) selectedText += ` | 색상: ${selectedColor}`;
    } else {
      selectedText += "❌ 선택 없음";
    }

    text(selectedText, startX, startY);
  }

  // ✅ 현재 단계별 UI 출력
  if (currentStep === 0) {
    text("🤖 어떤 기능을 사용할 건가요?", startX, startY + lineHeight);
    text("🖐️ 보자기 → 버튼 모드", startX, startY + 2 * lineHeight);
    text("✌️ 브이 → 슬라이더 모드", startX, startY + 3 * lineHeight);
  } else if (currentStep === 1) {
    text("🔘 어떤 버튼을 사용할 건가요?", startX, startY + lineHeight);
    text("🖐️ 보자기 → 버튼1", startX, startY + 2 * lineHeight);
    text("✌️ 브이 → 버튼2", startX, startY + 3 * lineHeight);
    text("👌 오케이 → 버튼3", startX, startY + 4 * lineHeight);
  } else if (currentStep === 2) {
    text("🎨 어떤 색을 조정할 건가요?", startX, startY + lineHeight);
    text("🖐️ 보자기 → 빨강", startX, startY + 2 * lineHeight);
    text("✌️ 브이 → 노랑", startX, startY + 3 * lineHeight);
    text("👌 오케이 → 초록", startX, startY + 4 * lineHeight);
  } else if (currentStep === 3) {
    text("🕒 신호 지속 시간을 조정합니다", startX, startY + lineHeight);
    text("☝️ 검지 위 → 증가", startX, startY + 2 * lineHeight);
    text("👇 검지 아래 → 감소", startX, startY + 3 * lineHeight);
  }

  // ✅ 뒤로 가기 안내 표시 (currentStep > 0 일 때만)
  if (currentStep > 0) {
    text("🤘 락앤롤 → 뒤로 가기", startX, startY + 5 * lineHeight);
  }
}

// 슬라이더 값 조절 (손 제스처)
function adjustSlider(color, change) {
  if (color.includes("red")) {
    print(color, change);

    redSlider.value(constrain(redSlider.value() + change, 0, 4000));
  } else if (color.includes("yellow")) {
    yellowSlider.value(constrain(yellowSlider.value() + change, 0, 4000));
  } else if (color.includes("green")) {
    greenSlider.value(constrain(greenSlider.value() + change, 0, 4000));
  }
  //sendData(); // 새로운 값 아두이노로 전송
}

// 손 관절에 원 그리기
function drawHandJoints() {
  for (let i = 0; i < hands.length; i++) {
    let hand = hands[i];
    for (let j = 0; j < hand.keypoints.length; j++) {
      let keypoint = hand.keypoints[j];
      fill(255, 0, 0);
      noStroke();
      circle(640 - keypoint.x, keypoint.y, 10); // 좌우 반전
    }
  }
}

function moveToNextStep() {
  canSelect = false; // 입력 잠금
  setTimeout(() => {
    currentStep++;
    canSelect = true; // 일정 시간 후 입력 가능
  }, 2000); // 1초 대기 후 다음 단계 이동
}

// 신호등 LED 표시
function drawTrafficLights() {
  noStroke();
  // LED 상태에 따라 색상 및 투명도 조절 (밝기)
  let redAlpha = redState ? brightness : 0;
  let yellowAlpha = yellowState ? brightness : 0;
  let greenAlpha = greenState ? brightness : 0;

  // R, Y,G LED 표시 (원형)
  fill(255, 0, 0, redAlpha);
  ellipse(800, 270, 50, 50);
  fill(0);
  textSize(20);
  textAlign(CENTER, CENTER);
  text("R", 800, 270);

  fill(255, 255, 0, yellowAlpha);
  ellipse(960, 270, 50, 50);
  fill(0);
  text("Y", 960, 270);

  fill(0, 255, 0, greenAlpha);
  ellipse(1120, 270, 50, 50);
  fill(0);
  text("G", 1120, 270);
}

// 밝기 게이지
function drawBrightnessGauge() {
  fill(255);
  stroke(0);
  strokeWeight(2);
  ellipse(1190, 420, 60, 60);

  noFill();
  stroke(0, 0, 255);
  strokeWeight(6);
  let angle = map(brightness, 0, 255, 0, TWO_PI);
  arc(1190, 420, 50, 50, -HALF_PI, -HALF_PI + angle);

  // 중앙 밝기 숫자 표시
  noStroke();
  fill(0); // 텍스트 색상
  textSize(16);
  textAlign(CENTER, CENTER);
  text(int(brightness), 1190, 420);
}

// 신호등 모드 표시
function drawModeIndicator() {
  let labels = ["Normal", "Button1", "Button2", "Button3"];
  let colors = ["gray", "gray", "gray", "gray"];
  colors[mode] = "blue";

  for (let i = 0; i < 4; i++) {
    fill(colors[i]);
    noStroke();
    rect(720 + i * 100, 380, 80, 40);
    fill(255);
    textSize(14);
    textAlign(CENTER, CENTER);
    text(labels[i], 760 + i * 100, 400);
  }
}

// 시리얼 데이터 처리
function processSerialData(data) {
  let values = data.split(",");
  if (values.length === 5) {
    redState = int(values[0]);
    yellowState = int(values[1]);
    greenState = int(values[2]);
    brightness = int(values[3]);
    //print(brightness);
    mode = int(values[4]);
  }
}

// 모드 변경 & 아두이노 전송
function sendModeToArduino(selectedMode) {
  if (port.opened()) {
    port.write("mode:" + selectedMode + "\n");
  }
}

function connectBtnClick() {
  if (!port.opened()) {
    port.open(9600);
  } else {
    port.close();
  }
}

function toggleMode(selectedMode) {
  if (mode === selectedMode) {
    mode = 0;
  } else {
    mode = selectedMode;
  }
  sendModeToArduino(mode);
}

function gotHands(results) {
  hands = results;
}

// 슬라이더 값 전송 (아두이노로)
function sendData() {
  if (port.opened()) {
    // 포트가 열려있을 때만 데이터 전송
    let msg =
      redSlider.value() +
      "," +
      yellowSlider.value() +
      "," +
      greenSlider.value() +
      "\n"; // 슬라이더 값 전송
    print(msg);
    port.write(msg); // 시리얼 포트로 데이터 전송
  }
}

function toggleButton(button, flag) {
  if (flag % 2 === 1) {
    currentAction = `${button} 활성화`; // 버튼 ON 상태
    mode = getModeByButton(button); // 해당 버튼에 맞는 모드 설정
  } else {
    currentAction = `${button} 비활성화`; // 버튼 OFF 상태
    mode = 0; // Normal 모드로 복귀
  }
  sendModeToArduino(mode); // 변경된 모드 아두이노로 전송
}

function getModeByButton(button) {
  switch (button) {
    case "button1":
      return 1; // Emergency Mode
    case "button2":
      return 2; // Blinking Mode
    case "button3":
      return 3; // On/Off Mode
    default:
      return 0; // 기본 모드 (Normal)
  }
}

// 👊 주먹 감지 (모든 손가락이 접힌 상태)
function isFist(landmarks) {
  return landmarks.slice(8, 21).every((p) => p.y > landmarks[0].y);
}

// ✌️ 브이 (검지 & 중지만 펴짐)
function isVSign(landmarks) {
  let indexUp = landmarks[8].y < landmarks[6].y;
  let middleUp = landmarks[12].y < landmarks[10].y;
  let ringDown = landmarks[16].y > landmarks[14].y;
  let pinkyDown = landmarks[20].y > landmarks[18].y;
  return indexUp && middleUp && ringDown && pinkyDown;
}

// 🖐️ 보자기 (손바닥 전체 펴짐)
function isPalm(landmarks) {
  return (
    landmarks[8].y < landmarks[6].y &&
    landmarks[12].y < landmarks[10].y &&
    landmarks[16].y < landmarks[14].y &&
    landmarks[20].y < landmarks[18].y
  );
}

// ☝️ 검지 위 (검지만 펴짐)
function isPointingUp(landmarks) {
  return (
    landmarks[8].y < landmarks[6].y && // 검지가 위쪽
    landmarks.slice(12, 20).every((p) => p.y > landmarks[8].y)
  ); // 나머지 손가락 접힘
}

// 👇 검지 아래 (검지만 아래)
function isPointingDown(landmarks) {
  return (
    landmarks[8].y > landmarks[6].y && // 검지가 아래쪽
    landmarks.slice(12, 20).every((p) => p.y < landmarks[8].y)
  ); // 나머지 손가락 펴짐
}

// 👍 엄지 위
function isThumbUp(landmarks) {
  return landmarks[4].y < landmarks[2].y && landmarks[4].x < landmarks[3].x;
}

// 👉 검지 옆으로 (손가락 가로로 이동)
function isIndexSideways(landmarks) {
  return (
    Math.abs(landmarks[8].x - landmarks[6].x) > 10 &&
    landmarks[8].y > landmarks[6].y
  );
}

// 🤘 락앤롤 제스처 감지 함수 (엄지 + 새끼손가락만 펴짐)
function isRockAndRoll(landmarks) {
  let thumbUp = landmarks[4].y < landmarks[2].y; // 엄지 펴짐
  let pinkyUp = landmarks[20].y < landmarks[18].y; // 새끼손가락 펴짐
  let otherFingersDown =
    landmarks[8].y > landmarks[6].y && // 검지 접힘
    landmarks[12].y > landmarks[10].y && // 중지 접힘
    landmarks[16].y > landmarks[14].y; // 약지 접힘

  return thumbUp && pinkyUp && otherFingersDown;
}

function isOKSign(landmarks) {
  // 거리 계산 (엄지 끝 4번, 검지 끝 8번)
  let thumbTip = landmarks[4];
  let indexTip = landmarks[8];
  let d = dist(thumbTip.x, thumbTip.y, indexTip.x, indexTip.y);

  // 기준 거리 (두 점이 붙어 있어야 함)
  let touching = d < 30;

  // 나머지 손가락은 펴져 있어야 함
  let middleStraight = landmarks[12].y < landmarks[10].y;
  let ringStraight = landmarks[16].y < landmarks[14].y;
  let pinkyStraight = landmarks[20].y < landmarks[18].y;

  return touching && middleStraight && ringStraight && pinkyStraight;
}
