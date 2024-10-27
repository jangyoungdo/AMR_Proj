#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <DueTimer.h>

// 핀 정의

// 리프트 초음파
#define TRIG_PIN 9
#define ECHO_PIN 10

#define MOTOR_PIN1 11
#define MOTOR_PIN2 12
#define D13_PIN 13  // D13 핀 정의
// #define IR_SENSOR_PIN 22  // 단일 IR 센서 핀 정의

// E18-D80NK 디지털 출력 핀 정의
const int e18_sensors[7] = {2, 3, 4, 5, 6, 7, 22};  // 6개의 E18-D80NK 디지털 출력 핀

volatile bool objectDetected = false;
// volatile bool singleobjectDetected = false;
volatile bool objectDetectedFlag = false;  // 객체 감지 플래그
// volatile bool singleSensorDetectedFlag = false;  // 단일 IR 센서 객체 감지 플래그

long duration;
int distanceMeasured;  // 리프트 높이

String message;
String E_message = "E_1";  // 평상시에는 E_1로 초기화

bool liftMovingFlag = false;  // 리프트 동작 플래그
int targetHeight = -1;  // 목표 높이 초기값
bool flag = false;
bool BPflag = false;

// Queue handle
QueueHandle_t distanceQueue;

// Task handles 
TaskHandle_t measureDistanceHandle;
TaskHandle_t objectDetectionHandle;
TaskHandle_t liftControlHandle;
TaskHandle_t messageReceiverHandle;
TaskHandle_t checkD13PinHandle;
// TaskHandle_t singleSensorObjectDetectionHandle;

// 함수 선언
void measureDistance(void *pvParameters);
void objectDetection(void *pvParameters);
void liftControl(void *pvParameters);
void messageReceiver(void *pvParameters);
void checkD13Pin(void *pvParameters);
// void singleSensorObjectDetection(void *pvParameters);
void JustLift_up();
void JustLift_down();
void liftUp(int targetHeight, int currentHeight);
void liftDown(int targetHeight, int currentHeight);
void liftStop();

void measureDistance(void *pvParameters)
{
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  for( ;; ) {
    // 첫 번째 초음파 센서에 펄스 전송
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // 첫 번째 센서의 펄스 지속 시간 측정
    duration = pulseIn(ECHO_PIN, HIGH);

    // 지속 시간을 거리로 변환 (cm)
    distanceMeasured = (duration * 0.034) / 2;

    // Queue에 거리 값 전송
    int distance = distanceMeasured;
    xQueueSend(distanceQueue, &distance, portMAX_DELAY);

    vTaskDelay(50 / portTICK_PERIOD_MS);  // 50ms 주기
  }
}


void objectDetection(void *pvParameters)
{
  for (int i = 0; i < 7; i++) {
    pinMode(e18_sensors[i], INPUT);
  }

  for( ;; ) {
    objectDetected = false;
    
    for (int i = 0; i < 7; i++) {
      int e18_state = digitalRead(e18_sensors[i]);
      if (e18_state == LOW) {  // 장애물 감지됨
        objectDetected = true;
        break;  // 하나의 센서에서 감지되면 더 이상 확인할 필요 없음
      }
    }

    if (objectDetected && !objectDetectedFlag) {
      if(flag == true){
        Serial.println("Object Detected");
      }
      E_message = "E_2";  // 객체 감지 시 E_message를 E_2로 설정
      Serial.println(E_message);
      objectDetectedFlag = true; // 객체 감지 플래그 설정
    } else if (!objectDetected && objectDetectedFlag) {
      // 모든 감지 플래그가 해제되었는지 확인
      if (!BPflag) {
        E_message = "E_3";  // 객체 감지 해제 시 E_message를 E_3로 설정
        Serial.println(E_message);
        objectDetectedFlag = false;  // 객체 감지 플래그 해제
      }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);  // 10ms 주기
  }
}

void liftControl(void *pvParameters)
{
  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);

  for( ;; ) {
    int currentHeight;
    if (xQueueReceive(distanceQueue, &currentHeight, portMAX_DELAY)) {
      if(flag == true){
        Serial.print("Current height: ");
        Serial.println(currentHeight);
      }
      if (liftMovingFlag && targetHeight != -1) {  // 유효한 목표 높이가 설정된 경우에만 리프트 동작
        if (currentHeight < targetHeight) {
          liftUp(targetHeight, currentHeight);
        } else if (currentHeight > targetHeight) {
          liftDown(targetHeight, currentHeight);
        }
      }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);  // 10ms 주기
  }
}


// 변수 리스트 길이 3짜리 추가 (초기값 10, 20, 30)
int savedHeights[3] = {10, 20, 30};  // 높이를 저장할 리스트

void messageReceiver(void *pvParameters)
{
  for( ;; ) {
    if (Serial.available() > 0) {
      message = Serial.readStringUntil('\n'); // 수신부
      if (message == "L_10") {
        JustLift_up();
      } else if (message == "L_11") {
        JustLift_down();
      } 
      // 저장된 높이로 이동하는 부분
      else if (message == "move_1") {
        targetHeight = savedHeights[0];  // 0번 인덱스에 저장된 높이로 이동
        liftMovingFlag = true;
      } else if (message == "move_2") {
        targetHeight = savedHeights[1];  // 1번 인덱스에 저장된 높이로 이동
        liftMovingFlag = true;
      } else if (message == "move_3") {
        targetHeight = savedHeights[2];  // 2번 인덱스에 저장된 높이로 이동
        liftMovingFlag = true;
      }
      // 초음파 센서로 측정된 높이를 리스트에 저장하는 부분
      else if (message == "save_1") {
        savedHeights[0] = distanceMeasured;  // 0번 인덱스에 현재 측정된 높이 저장
        // Serial.print("Height saved at index 0: ");
        // Serial.println(savedHeights[0]);
      } else if (message == "save_2") {
        savedHeights[1] = distanceMeasured;  // 1번 인덱스에 현재 측정된 높이 저장
        // Serial.print("Height saved at index 1: ");
        // Serial.println(savedHeights[1]);
      } else if (message == "save_3") {
        savedHeights[2] = distanceMeasured;  // 2번 인덱스에 현재 측정된 높이 저장
        // Serial.print("Height saved at index 2: ");
        // Serial.println(savedHeights[2]);
      } 
      // 객체 감지 플래그 처리
      else if (message == "E_1") {
        E_message = "E_1";  // E_1 메시지를 수신하면 E_message를 E_1로 설정
        Serial.println(E_message);
        objectDetectedFlag = false;  // 객체가 감지되지 않을 때 플래그 해제
        BPflag = false;
      } 
      // 기타 동작을 방지하는 초기화
      else {
        liftMovingFlag = false;
        targetHeight = -1;  // 목표 높이를 초기화하여 불필요한 동작 방지
      }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);  // 10ms 주기
  }
}


void checkD13Pin(void *pvParameters)
{
  pinMode(D13_PIN, INPUT_PULLUP);  // 풀업 저항 설정

  for( ;; ) {
    if (digitalRead(D13_PIN) == LOW && !objectDetectedFlag) {
      BPflag = true;
      E_message = "E_0";  // 객체 감지 시 E_message를 E_0로 설정
      Serial.println(E_message);
      objectDetectedFlag= true;  // 객체 감지 플래그 설정
    }
    
    vTaskDelay(50 / portTICK_PERIOD_MS);  // 50ms 주기
  }
}

void JustLift_up() {
    digitalWrite(MOTOR_PIN1, HIGH);
    digitalWrite(MOTOR_PIN2, LOW);
    delay(500); // 필요한 경우 모터를 일정 시간 동안 동작시키기 위해 지연을 추가
    liftStop();
    liftMovingFlag = false;  // 동작 종료 후 플래그 초기화
    targetHeight = -1;  // 목표 높이 초기화
}

void JustLift_down() {
    digitalWrite(MOTOR_PIN1, LOW);
    digitalWrite(MOTOR_PIN2, HIGH);
    delay(500); // 필요한 경우 모터를 일정 시간 동안 동작시키기 위해 지연을 추가
    liftStop();
    liftMovingFlag = false;  // 동작 종료 후 플래그 초기화
    targetHeight = -1;  // 목표 높이 초기화
}

void liftUp(int targetHeight, int currentHeight) {
  while (currentHeight < targetHeight) {
    digitalWrite(MOTOR_PIN1, HIGH);
    digitalWrite(MOTOR_PIN2, LOW);

    if (xQueueReceive(distanceQueue, &currentHeight, portMAX_DELAY)) {
      if(flag == true){
        Serial.print("Current height (lifting up): ");
        Serial.println(currentHeight);
      }
    }

    if (currentHeight >= targetHeight) {
      liftMovingFlag = false;  // 목표 높이에 도달하면 플래그를 false로 설정
      targetHeight = -1;  // 목표 높이 초기화
      break;
    }
  }
  liftStop();
}

void liftDown(int targetHeight, int currentHeight) {
  while (currentHeight > targetHeight) {
    digitalWrite(MOTOR_PIN1, LOW);
    digitalWrite(MOTOR_PIN2, HIGH);

    if (xQueueReceive(distanceQueue, &currentHeight, portMAX_DELAY)) {
      if(flag == true){
        Serial.print("Current height (lifting down): ");
        Serial.println(currentHeight);
      }
    }

    if (currentHeight <= targetHeight) {
      liftMovingFlag = false;  // 목표 높이에 도달하면 플래그를 false로 설정
      targetHeight = -1;  // 목표 높이 초기화
      break;
    }
  }
  liftStop();
}

void liftStop() {
  digitalWrite(MOTOR_PIN1, LOW);
  digitalWrite(MOTOR_PIN2, LOW);
}

void setup() 
{
  Serial.begin(115200);

  // Queue 생성
  distanceQueue = xQueueCreate(10, sizeof(int));

  if (distanceQueue == NULL) {
    while (1);
  }

  xTaskCreate(measureDistance, (const portCHAR *)"measureDistance", 256, NULL, 3, &measureDistanceHandle);  // 초음파 거리 측정 태스크
  xTaskCreate(objectDetection, (const portCHAR *)"objectDetection", 256, NULL, 6, &objectDetectionHandle);  // 객체 감지 태스크
  // xTaskCreate(singleSensorObjectDetection, (const portCHAR *)"singleSensorObjectDetection", 256, NULL, 6, &singleSensorObjectDetectionHandle);  // 단일 IR 센서 객체 감지 태스크
  xTaskCreate(liftControl, (const portCHAR *)"liftControl", 256, NULL, 3, &liftControlHandle);  // 리프트 제어 태스크
  xTaskCreate(messageReceiver, (const portCHAR *)"messageReceiver", 256, NULL, 4, &messageReceiverHandle);  // 메시지 수신 태스크
  xTaskCreate(checkD13Pin, (const portCHAR *)"checkD13Pin", 256, NULL, 6, &checkD13PinHandle);  // 범퍼 스위치 D13 핀 신호 확인 태스크
  
  vTaskStartScheduler();

  while(1);
}

void loop()
{
  // 아무 작업도 하지 않음
}
