// 손 인식을 위한 변수
let handPose;
let video;
let hands = [];

// 4가지 모드를 위한 변수 
let modes = ["good", "sad", "ok", "image"];
let activeMode = null;
let modeBoxes = [];

// 이미지 업로드를 위한 변수
let uploadedImg;
let imgUploadBtn;

let hslValue = 0; // 색상 선택을 위한 변수
let emojiEffects = []; // 이모지 이펙트를 위한 변수
let warningTextTimer = 0; // 이미지 업로드 경고를 위한 변수수
let sliderX = 320; // 펜 색상 변경을 위한 변수 
let drawings = []; // 모든 선들 모음
let currentPath = []; // 현재 그리고 있는 선 (점들의 배열)
let strokeThickness = 4; // 두께조절
let isImageModeOn = false; // 이미지 모드 토글 변수 

// 그리기, 지우기 관련 변수 
let isDrawing = false;
let isErasing = false;

//ml5.js 손 인식 모델 로드
function preload() {
  handPose = ml5.handPose();
}

// 손 인식 결과를 받아오는 콜백 함수
function gotHands(results) {
  hands = results;
}

function setup() {
  createCanvas(640, 600); // 480(video) + 100(ui)
  colorMode(HSL); // ✅ 색상 모드를 HSL로 변경
  noSmooth(); // 🔧 픽셀 선명하게!

  video = createCapture(VIDEO, { flipped: true }); // ✅ 좌우 반전  
  video.size(640, 480);
  video.hide();
  handPose.detectStart(video, gotHands); // 비디오 속 손 인식 시작

  // 모드 박스 초기화
  const boxSize = 60;
  for (let i = 0; i < modes.length; i++) {
    modeBoxes.push({
      x: width / 2 - 140 + i * (boxSize + 10),
      y: 10,
      w: boxSize,
      h: boxSize,
      mode: modes[i],
      active: false,
    });
  }

  // 이미지 업로드 버튼
  imgUploadBtn = select("#imgUpload");
  imgUploadBtn.changed(handleImageUpload);
  imgUploadBtn.position(430, 572); // ← 색상바 오른쪽 아래에 고정
}

function draw() {
  background(255);
  image(video, 0, 0, 640, 480);
  drawHandPointer(); // 손가락 포인터 그리기
  drawModeBoxes(); // 모드 박스 그리기
  drawHSLBar(); // 색상 슬라이더 그리기
  handleHandInteraction(); // 손가락으로 모드 박스 클릭
  drawEmojiEffects(); // 이모지 이펙트 그리기
  drawDrawings(); // 그림 그리기
  handleDrawingGesture(); // 그리기 제스처
  handleErasingGesture(); // 지우기 제스처
  handleRockGesture(); // 🤘 락앤롤 제스처 - 그림 전체 지우기 
  updateStrokeThicknessFromLeftHand(); // 왼손으로 선 두께 조절
  updateHSLByThumbPinky_X(); // 오른손으로 색상 선택

  // ⚠️ 업로드 경고 텍스트 (2초 동안 표시) - image 모드일 때만
  if (warningTextTimer > 0) {
    noStroke();
    drawWarningText("업로드 된 이미지가 없습니다");
    warningTextTimer--;
  }

  // 🖼️ 이미지 모드일 때 손바닥에 이미지 그리기
  if (isImageModeOn && uploadedImg) {
    drawImageOnPalms();
  }
}

// ✨ 그림 그리기 관련 함수들
function drawDrawings() {
  noFill();
  strokeJoin(ROUND); // ✅ 코너 부드럽게
  strokeCap(ROUND); // ✅ 선 끝도 깔끔하게

  for (let path of drawings) { // 저장된 선들 그리기
    beginShape(); // 여러 점들을 이어서 선으로 그리기
    for (let pt of path) { // 저장된 점들을 이어서 선으로 그리기
      stroke(pt.color);
      strokeWeight(pt.thickness || 4); // 저장된 두께 사용
      vertex(pt.x, pt.y);
    }
    endShape();  // 여러 점들을 이어서 선으로 그리기
  }

  if (currentPath.length > 0) { // 현재 그리는 선 그리기
    beginShape();
    for (let pt of currentPath) { 
      stroke(pt.color);
      strokeWeight(pt.thickness || 4);
      vertex(pt.x, pt.y);
    }
    endShape();
  }
}

// ✨ 그리기 제스처 관련 함수들
function handleDrawingGesture() {
  if (hands.length === 0) return; 
  let hand = hands[0];
  let points = hand.keypoints; // 손가락 관절들

  // 검지 끝, 엄지 끝 찾기
  let index = points.find((kp) => kp.name === "index_finger_tip");
  let thumb = points.find((kp) => kp.name === "thumb_tip");

  if (!index || !thumb) return;

  // 거리 계산
  let d = dist(index.x, index.y, thumb.x, thumb.y);

  if (d < 20) {
    // 손가락 붙은 상태
    let drawX = width - index.x;
    let drawY = index.y;

    // 그리기: 새로운 점 추가
    let point = {
      x: drawX,
      y: drawY,
      color: color(hslValue, 100, 50), //  HSL 기반 p5 Color 객체
      thickness: strokeThickness,
    };

    currentPath.push(point);
    isDrawing = true;
  } else if (isDrawing) {
    // 손가락 뗐을 때 선 저장
    drawings.push(currentPath);
    currentPath = [];
    isDrawing = false;
  }
}

// ✨ 지우기 제스처 관련 함수들
function handleErasingGesture() {
  if (hands.length === 0) return;
  let hand = hands[0];
  let points = hand.keypoints;

  // 엄지 끝, 중지 끝 찾기
  let thumb = points.find((kp) => kp.name === "thumb_tip");
  let middle = points.find((kp) => kp.name === "middle_finger_tip");

  if (!thumb || !middle) return;

  // 거리 계산
  let d = dist(thumb.x, thumb.y, middle.x, middle.y);

  if (d < 26) {
    isErasing = true;

    let ex = width - thumb.x;
    let ey = thumb.y;

    // 지우기: 가까운 점들 제거
    for (let i = 0; i < drawings.length; i++) {
      drawings[i] = drawings[i].filter((pt) => { // 필터링
        return dist(pt.x, pt.y, ex, ey) > 20;
      });
    }
  } else {
    isErasing = false;
  }

  if (isErasing) {
    fill(255, 0, 0, 100);
    noStroke();
    circle(width - thumb.x, thumb.y, 20);
  }
}

// ✨ 락앤롤 제스처 관련 함수들
function handleRockGesture() {
  if (hands.length === 0) return;

  const lm = hands[0].keypoints; // ✅ lm을 여기서 정의

  if (isRockAndRoll(lm)) {
    currentGesture = "🤘 락앤롤";
    drawings = [];
    currentPath = [];
    console.log("전체 그림 삭제됨!");
  }
}

// ✨ 락앤롤 제스처 판단 함수
function isRockAndRoll(lm) {
  return (
    isFingerExtended(lm, 4) && // 엄지 펼침
    isFingerExtended(lm, 20) && // 손가락 펼침
    !isFingerExtended(lm, 8) && // 검지 접힘
    !isFingerExtended(lm, 12) &&  // 중지 접힘
    !isFingerExtended(lm, 16) // 약지 접힘 
  );
}

// ✨ 손가락 펼침 여부 판단 함수
function isFingerExtended(lm, tipIndex) {
  const tip = lm[tipIndex]; // 끝 관절
  const pip = lm[tipIndex - 2]; // 중간 관절

  return tip.y < pip.y; // 위쪽에 있으면 펼친 거
}

// ✨ 왼손으로 선 두께 조절
function updateStrokeThicknessFromLeftHand() {
  if (hands.length < 2) return; // 두 손 필요!

  let left = hands[1].keypoints;

  // 엄지 끝, 검지 끝 찾기
  let thumb = left.find((kp) => kp.name === "thumb_tip");
  let index = left.find((kp) => kp.name === "index_finger_tip");

  if (!thumb || !index) return;

  let d = dist(thumb.x, thumb.y, index.x, index.y);

  // 거리 → 두께 매핑
  strokeThickness = map(d, 10, 150, 2, 20); // 10~150 → 2~20
  strokeThickness = constrain(strokeThickness, 2, 20); // 2~20 사이로 제한

  // 좌우 반전 좌표 (화면에 맞게)
  let x1 = width - thumb.x;
  let y1 = thumb.y;
  let x2 = width - index.x;
  let y2 = index.y;

  // ✨ 선 그리기 (초록색)
  if (isDrawing) {
    push();
    stroke(120, 100, 40); // 초록색 (HSL 기준)
    strokeWeight(strokeThickness);
    line(x1, y1, x2, y2);
    pop();
  }
}

// ✨ 오른손으로 색상 선택
function updateHSLByThumbPinky_X() {
  if (hands.length === 0) return;

  const rightHand = hands[0].keypoints;
  // 엄지 끝, 약지 끝 찾기
  const thumbTip = rightHand.find((kp) => kp.name === "thumb_tip");
  const ringTip = rightHand.find((kp) => kp.name === "ring_finger_tip"); // ✅ 수정됨

  if (!thumbTip || !ringTip) return;

  const d = dist(thumbTip.x, thumbTip.y, ringTip.x, ringTip.y);
  if (d < 25) {
    const x = width - ringTip.x;
    // ✅ x 좌표를 HSL 값으로 변환
    hslValue = map(x, 0, width, 0, 360);
    hslValue = constrain(hslValue, 0, 360);
    sliderX = x;

    sliderX = map(hslValue, 0, 360, 0, width); // ✅ 슬라이더 위치 동기화

    // 색상 미리보기 (엄지 위)
    push();
    let c = color(hslValue, 100, 50); // ← HSL 모드에서 색 객체
    fill(c);
    noStroke();
    let cx = width - thumbTip.x;
    let cy = thumbTip.y - 25;
    circle(cx, cy, 18);
    pop();

    console.log("🌈색상 변경됨! hue:", hslValue);
  }
}

// 손가락 포인터 그리기 (검지 끝, 엄지 끝)
function drawHandPointer() {
  for (let hand of hands) {
    if (!hand.keypoints || hand.keypoints.length < 9) continue;

    // 검지 끝 (indexFinger[4]), 엄지 끝 (thumb[4])
    let indexTip = hand.keypoints.find((kp) => kp.name === "index_finger_tip");
    let thumbTip = hand.keypoints.find((kp) => kp.name === "thumb_tip");

    // 검지 끝에 원 그리기
    if (indexTip) {
      noStroke();

      fill("red");
      circle(width - indexTip.x, indexTip.y, 10); // 좌우 반전
    }

    // 엄지 끝에 원 그리기
    if (thumbTip) {
      noStroke();

      fill("blue");
      circle(width - thumbTip.x, thumbTip.y, 10); // 좌우 반전
    }
  }
}

// 🟦 모드 박스 UI - 활성화 박스: blue, 비활성화 박스: gray 
function drawModeBoxes() {
  for (let box of modeBoxes) {
    noStroke();
    fill(box.active ? "blue" : "gray");
    rect(box.x, box.y, box.w, box.h, 10);

    textAlign(CENTER, CENTER);
    textSize(24);
    let emoji = { good: "👍", sad: "😢", ok: "👌", image: "🖼️" }[box.mode];
    fill(255);
    text(emoji, box.x + box.w / 2, box.y + box.h / 2);
  }
}

// 색상 슬라이더 UI 
function drawHSLBar() {
  for (let x = 0; x < width; x++) {
    stroke(map(x, 0, width, 0, 360), 100, 50);
    line(x, 490, x, 550);
  }
  noStroke();
  strokeWeight(2);
  line(sliderX, 490, sliderX, 550);

  let c = color(hslValue, 100, 50); // ← HSL 모드에서 색 객체

  // 🎯 선택된 색상 사각형 미리보기
  fill(c);
  rect(10, 570, 40, 30);
  fill(0);
  textAlign(LEFT, CENTER);
  text("색상 선택", 60, 585);

  // 슬라이더 위 삼각형 포인터
  push();
  fill(c);
  noStroke();
  triangle(sliderX, 488, sliderX - 6, 480, sliderX + 6, 480); // ▼ 삼각형
  pop();
}

// 🖱️ 검지로 모드 박스 클릭 인식
function handleHandInteraction() {
  if (hands.length === 0) return;
  let landmarks = hands[0].keypoints;
  // 손가락이 인식되지 않으면 종료
  if (!landmarks || landmarks.length < 9) return;

  // 검지 끝 찾기
  let indexTip = landmarks.find((kp) => kp.name === "index_finger_tip");
  if (!indexTip) return;

  let ix = width - indexTip.x; // 좌우 반전된 비디오에 맞추기
  let iy = indexTip.y;

  // 모드 박스 클릭 인식
  for (let box of modeBoxes) {
    if (
      !box.active &&
      ix > box.x &&
      ix < box.x + box.w &&
      iy > box.y &&
      iy < box.y + box.h
    ) {
      box.active = true;
      activeMode = box.mode;
      triggerModeAction(box.mode);
      setTimeout(() => {
        box.active = false;
      }, 2000); // 2초 후 비활성화
    }
  }
}

// 📦 모드에 따라 동작
function triggerModeAction(mode) {
  if (mode === "good" || mode === "sad" || mode === "ok") {
    let emoji = { good: "👍", sad: "😢", ok: "👌" }[mode];

    for (let i = 0; i < 10; i++) {
      let x = random(50, width - 50);
      let y = random(-100, 0);
      let speed = random(1, 4);
      let waveOffset = random(TWO_PI); // 각 이모지마다 다른 흔들림 위상
      emojiEffects.push({
        emoji,
        x,
        baseX: x,
        y,
        alpha: 255,
        speed,
        waveOffset,
      });
    }
  }

  //  이미지 모드
  if (mode === "image") {
    if (!uploadedImg) { // 이미지 없으면 경고 표시
      warningTextTimer = 120;
      isImageModeOn = false;
    } else {
      isImageModeOn = !isImageModeOn; // ✅ 토글 방식
      console.log("🖼️ 이미지 모드 상태:", isImageModeOn);
    }
  } else {
    isImageModeOn = false; // 다른 모드 누르면 끔
  }
}

// 이모지 이펙트 그리기
function drawEmojiEffects() {
  for (let i = emojiEffects.length - 1; i >= 0; i--) {
    let e = emojiEffects[i];

    // 흔들림 계산: baseX + sin파형
    let wave = sin(frameCount * 0.1 + e.waveOffset) * 10;
    let drawX = e.baseX + wave;

    fill(0, e.alpha);
    textSize(48);
    textAlign(CENTER, CENTER);
    text(e.emoji, drawX, e.y);

    // 이모지 이펙트 상태 업데이트
    e.y += e.speed;
    e.alpha -= 2;

    // 사라진 이모지 제거
    if (e.alpha <= 0 || e.y > height) {
      emojiEffects.splice(i, 1);
    }
  }
}

// 경고 텍스트 그리기
function drawWarningText(msg) {
  textSize(20);
  textAlign(CENTER, CENTER);
  let x = width / 2;
  let y = height / 2;

  // 배경 박스
  rectMode(CENTER); // 중앙 기준
  fill(0, 180); // 반투명 검정
  rect(x, y - 50, textWidth(msg) + 40, 50, 10);

  rectMode(CORNER); // 다시 원래대로 복구!

  // 텍스트
  fill(255);
  text(msg, x, y - 50);
}

// 이미지 업로드
function handleImageUpload() {
  // 업로드한 이미지를 p5.js 이미지로 로드
  const file = imgUploadBtn.elt.files[0];
  if (file) {
    // 이미지 로드
    loadImage(URL.createObjectURL(file), (img) => {
      uploadedImg = img;
      console.log("✅ 이미지 업로드 완료!");
    });
  }
}

// 손바닥 판단하기 - 손바닥이 열려있는지
function isOpenPalm(hand) {
  if (!hand || !hand.keypoints) return false;

  const lm = hand.keypoints;

  // 손바닥 여닫힘 판단
  const extendedFingers = [8, 12, 16, 20].every((i) => lm[i].y < lm[i - 2].y);
  const thumbExtended = Math.abs(lm[4].x - lm[3].x) > 10; // 약간만 펼쳐도 인정

  return extendedFingers && thumbExtended;
}

// 손바닥에 이미지 그리기
function drawImageOnPalms() {
  imageMode(CENTER); // 손바닥 이미지만 CENTER로
    for (let hand of hands) {
    if (isOpenPalm(hand)) {
      // 손바닥 중심 좌표
      const wrist = hand.keypoints.find(k => k.name === "wrist");
      if (wrist && uploadedImg) {
        let cx = width - wrist.x;
        let cy = wrist.y;

        // 유동적 크기 조절
        const maxSize =400; // 최대 너비나 높이

        let imgW = uploadedImg.width;
        let imgH = uploadedImg.height;

        let scale = 1;

        // 가로든 세로든 400 넘어가면 축소
        if (imgW > maxSize || imgH > maxSize) {
          scale = min(maxSize / imgW, maxSize / imgH);
        }
        // 확대된 크기
        let displayW = imgW * scale;
        let displayH = imgH * scale;

        push();
        imageMode(CENTER);
        image(uploadedImg, cx, cy, displayW, displayH);
        pop();
      }
    }
  }
  imageMode(CORNER); //  다시 원래대로 복구!
}
