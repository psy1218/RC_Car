let port; // 시리얼 포트 객체
// 신호등 LED 상태 및 밝기 변수
let redState = 0, yellowState = 0, greenState = 0, brightness = 0, mode = 0;
let redSlider, yellowSlider, greenSlider; // LED 지속 시간 조절 슬라이더
let connectBtn; // 연결 버튼
let modeButtons = []; // 모드 변경 버튼

function setup() {
  // 캔버스 생성
  createCanvas(500, 380);
  background(220);

  // 시리얼 포트 객체 생성
  port = createSerial(); // Web Serial 컨트롤 객체

  // 이전 연결 정보를 통해 자동으로 연결
  let usedPorts = usedSerialPorts();
  if (usedPorts.length > 0) {
    port.open(usedPorts[0], 9600); // 과거에 사용했던 포트에 9600 baud rate로 연결
  }

  // Web Serial 연결 버튼 생성
  connectBtn = createButton("Connect to Arduino");
  connectBtn.position(180, 10);
  connectBtn.mousePressed(connectBtnClick);

    // 모드 변경 버튼 추가
  let modeLabels = ["Normal", "Button1", "Button2", "Button3"];
  for (let i = 0; i < 4; i++) { // 4개의 버튼 생성
    modeButtons[i] = createButton(modeLabels[i]); // 버튼 레이블
    modeButtons[i].position(60 + i * 100, 340); // 버튼 위치
    modeButtons[i].mousePressed(() => toggleMode(i)); // 두 번 클릭하면 Normal로 돌아가도록 수정
  }
  
  // 슬라이더 생성 (LED 지속 시간 조절)
  redSlider = createSlider(0, 4000, 2000); // 최소 시간값, 최대 시간 값 , 초기값
  redSlider.position(35, 220); // 슬라이더 위치
  redSlider.input(sendData); // 슬라이더 값 변경 시 sendData() 함수 호출

  yellowSlider = createSlider(0, 4000, 500);
  yellowSlider.position(180, 220);
  yellowSlider.input(sendData);

  greenSlider = createSlider(0, 4000, 2000);
  greenSlider.position(330, 220);
  greenSlider.input(sendData);
}

function draw() {
  // 캔버스 초기화
  background(220);

  // 시리얼 데이터 수신 (draw() 내부에서 처리)
  if (port.available()) { // 데이터 수신 시
    let data = port.readUntil("\n"); // 한 줄 단위로 읽기
    processSerialData(data.trim()); // 개행문자 제거 후 처리
  }
  
  // 신호등 LED 상태 표시 (실시간 반영)
  drawTrafficLights();
  
  // 신호 길이 슬라이더 및 값 표시
  textSize(14);
  fill(0);
  // 텍스트 표시
  text("0s", 40, 250);
  text("4s", 160, 250);
  text("0s", 190, 250);
  text("4s", 310, 250);
  text("0s", 340, 250);
  text("4s", 460, 250);

  drawBrightnessGauge(); // 밝기 게이지 표시
  drawModeIndicator();  // 모드 인디케이터 표시
}

// 신호등 LED 상태를 실시간 반영하여 표시하는 함수
function drawTrafficLights() {
  noStroke();
  // LED 상태에 따라 색상 및 투명도 조절 (밝기)
  let redAlpha = redState ? brightness : 0;    
  let yellowAlpha = yellowState ? brightness : 0;
  let greenAlpha = greenState ? brightness : 0;

  // R, Y,G LED 표시 (원형)
  fill(255, 0, 0, redAlpha);
  ellipse(100, 150, 50, 50);
  fill(0);
  textSize(20);
  textAlign(CENTER, CENTER);
  text("R", 100, 150);

  fill(255, 255, 0, yellowAlpha);
  ellipse(245, 150, 50, 50);
  fill(0);
  text("Y", 245, 150);

  fill(0, 255, 0, greenAlpha);
  ellipse(390, 150, 50, 50);
  fill(0);
  text("G", 390, 150);
}


// 신호등 모드 인디케이터
function drawModeIndicator() {
  let labels = ["Normal", "Button1", "Button2", "Button3"]; // 모드 레이블
  let colors = ['gray', 'gray', 'gray', 'gray']; 
  colors[mode] = 'blue'; // 선택된 모드만 파란색으로 표시

  // 모드 버튼 표시
  for (let i = 0; i < 4; i++) {
    fill(colors[i]);
    strokeWeight(1);
    stroke(0, 0, 0); 
    rect(50 + i * 100, 290, 80, 40);

    fill(255);
    textSize(14);
    textAlign(CENTER, CENTER);
    text(labels[i], 90 + i * 100, 310);
  }
}

// 밝기 인디케이터 (원형 게이지)
function drawBrightnessGauge() {
  fill(255);
  stroke(0);
  strokeWeight(2);
  ellipse(245, 80, 60, 60); // 원형 게이지 배경

  noFill();
  stroke(0, 0, 255);
  strokeWeight(6);
  let angle = map(brightness, 0, 255, 0, TWO_PI); // 밝기 값에 따른 각도 계산
  arc(245, 80, 50, 50, -HALF_PI, -HALF_PI + angle); // 밝기 값에 따른 부채꼴 그리기
}

// 시리얼 데이터 처리 함수
function processSerialData(data) {  
  let values = data.split(","); // 쉼표로 구분하여 배열로 변환
  if (values.length === 5) { // 데이터 길이가 5인 경우만 처리
    // 데이터를 정수형으로 변환하여 변수에 저장
    redState = int(values[0]);
    yellowState = int(values[1]);
    greenState = int(values[2]);
    brightness = int(values[3]);
    mode = int(values[4]);
  }
}

// 슬라이더 값 전송 (아두이노로)
function sendData() {
  if (port.opened()) { // 포트가 열려있을 때만 데이터 전송
    let msg = redSlider.value() + "," + yellowSlider.value() + "," + greenSlider.value() + "\n";// 슬라이더 값 전송
    port.write(msg);  // 시리얼 포트로 데이터 전송
  }
}

// 모드 변경 버튼을 두 번 누르면 Normal(0)로 복귀
function toggleMode(selectedMode) {
  if (mode === selectedMode) {
    mode = 0; // 현재 모드와 같으면 Normal(0)로 복귀
  } else {
    mode = selectedMode; // 새로운 모드 적용
  }
  sendModeToArduino(mode); // 아두이노로 모드 전송
}

function sendModeToArduino(selectedMode) {
  if (port.opened()) {  // 포트가 열려있을 때만 데이터 전송
    port.write("mode:" + selectedMode + "\n");  // 모드 변경 메시지 전송
  }
}

// Arduino 연결/해제 버튼 동작
function connectBtnClick() {
  if (!port.opened()) {
    port.open(9600); // 9600 baudRate
  } else {
    port.close();
  }
}

