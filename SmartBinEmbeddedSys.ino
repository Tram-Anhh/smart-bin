#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>

static const uint8_t PIN_MP3_TX = 26; // Connects to module's RX 
static const uint8_t PIN_MP3_RX = 27; // Connects to module's TX 
SoftwareSerial softwareSerial(PIN_MP3_RX, PIN_MP3_TX);

// Create the Player object
DFRobotDFPlayerMini player;
#define RELAY_PIN 13
// Ultrasonic sensor pins
const int trigger1 = 4;
const int echo1 = 2;

const int trigger2 = 14;
const int echo2 = 12;

// Servo configuration
Servo myservo;
#define servoPin 15
int pos = 0;
int age = 90;
bool lidOpen = false;
bool alertPlayed = false; 
// LCD configuration
LiquidCrystal_I2C lcd(0x27, 24, 3); // Adjust the I2C address if needed

// FreeRTOS queue and semaphores
QueueHandle_t distanceQueue;
SemaphoreHandle_t xBinarySemaphore;
SemaphoreHandle_t xMutex;

// Structure to hold distance data
struct DistanceData {
    float distance;
    float newDistance;
};

// Function to measure distance using an ultrasonic sensor
float measureDistance(int triggerPin, int echoPin) {
    long duration;
    float distance;

    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);

    duration = pulseIn(echoPin, HIGH);
    distance = duration / 2.0 / 29.412;

    return distance;
}

// Task to control the servo motor
void servoControl(void *pvParameters) {
    DistanceData distanceData;
    for (;;) {
        if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdTRUE) {
            if (xQueueReceive(distanceQueue, &distanceData, portMAX_DELAY) == pdTRUE) {
                if (distanceData.newDistance < 5 || distanceData.newDistance > 25) {
                    if (!lidOpen) {
                        player.play(5);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                        digitalWrite(RELAY_PIN, HIGH); // Bật relay
                        delay(5000); // Chờ 1 giây
                        digitalWrite(RELAY_PIN, LOW); // Tắt relay
                        lidOpen = true;
                        lcd.clear();
                        //lcd.setCursor(2, 0);
                        //lcd.print("  `    '   ");
                        lcd.setCursor(5,0);
                        lcd.print("Bin is full");
                        lcd.setCursor(2,2);
                        lcd.print("inner distance: ");
                        lcd.setCursor(7, 3);
                        lcd.print(distanceData.newDistance);
                        lcd.print(" cm");
                            }
                } else if (distanceData.newDistance >= 5 && distanceData.distance < 15) {

                    lidOpen = false;
                    lcd.clear();
                    //lcd.setCursor(4, 0);
                    //lcd.print("~   ?  '");
                    lcd.setCursor(3, 0);
                    lcd.print("Give me trash");
                    lcd.setCursor(1, 2);
                    lcd.print("external distance: ");
                    lcd.setCursor(7, 3);
                    lcd.print(distanceData.distance);
                    lcd.print(" cm");
                    player.play(6);
                    
                    myservo.attach(servoPin, 544, 2450);
                    if (pos == 0) {
                        for (pos = 0; pos <= age; pos += 2) {
                            myservo.write(pos);
                            vTaskDelay(pdMS_TO_TICKS(20));
                        }
                        pos = age;
                        vTaskDelay(pdMS_TO_TICKS(5000));
                    }
                    if (pos == age) {
                        for (pos = age; pos >= 0; pos -= 2) {
                            myservo.write(pos);
                            vTaskDelay(pdMS_TO_TICKS(20));
                        }
                        pos = 0;
                        myservo.detach();
                    }
                } else if (distanceData.distance >= 15) {
                        lcd.clear();
                    //lcd.setCursor(4, 0);
                    //lcd.print("~   ?  '");
                         lcd.setCursor(3,0);
                         lcd.print("Bin is closed");
                         lcd.setCursor(1,2);
                        lcd.print("external distance: ");
                        lcd.setCursor(7, 3);
                        lcd.print(distanceData.distance);
                        lcd.print("cm");
                    if (lidOpen) {
                        myservo.attach(servoPin, 544, 2450);
                        for (pos = age; pos >= 0; pos -= 2) {
                            myservo.write(pos);
                            vTaskDelay(pdMS_TO_TICKS(20));
                        }
                        pos = 0;
                        myservo.detach();
                        lidOpen = false;
                        
                    }
                }
                
            }
        }
    }
}

// Task to measure distances from ultrasonic sensors
void distanceMeasurement(void *pvParameters) {
    DistanceData distanceData;
    for (;;) {
        pinMode(trigger1, OUTPUT);
        pinMode(echo1, INPUT);
        distanceData.distance = measureDistance(trigger1, echo1);

        vTaskDelay(pdMS_TO_TICKS(50));

        pinMode(trigger2, OUTPUT);
        pinMode(echo2, INPUT);
        distanceData.newDistance = measureDistance(trigger2, echo2);

        if (xQueueSendToBack(distanceQueue, &distanceData, portMAX_DELAY) != pdPASS) {
            Serial.println("Failed to send to queue.");
        } else {
            xSemaphoreGive(xBinarySemaphore);
        }

        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

// Task to continuously print messages to the Serial Monitor and LCD


// Task to handle receiving and printing distance data
void receiverTask(void *pvParameters) {
    DistanceData receivedDistanceData;
    for (;;) {
        if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdTRUE) {
            if (xQueueReceive(distanceQueue, &receivedDistanceData, portMAX_DELAY)) {
                if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
                    Serial.print("Khoảng cách ngoài: ");
                    Serial.print(receivedDistanceData.distance);
                    Serial.print(" cm, Khoảng cách trong: ");
                    Serial.print(receivedDistanceData.newDistance);
                    Serial.println(" cm");

                    xSemaphoreGive(xMutex);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

void checkDistanceAndPlaySound(void *pvParameters) {
    DistanceData distanceData;
    for (;;) { // Vòng lặp vô hạn
        if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdTRUE) {
            if (xQueueReceive(distanceQueue, &distanceData, portMAX_DELAY) == pdTRUE) {
                if (distanceData.newDistance < 5 && distanceData.distance < 15||distanceData.newDistance > 50 && distanceData.distance < 15) {
                    if (!alertPlayed) {
                        player.play(5);
                        alertPlayed = true; // Cập nhật trạng thái để không báo lại ngay lập tức
                    }
                } else {
                    alertPlayed = false; // Reset trạng thái khi điều kiện không còn đúng
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Tạm dừng 100ms trước khi kiểm tra lại
    }
}
void setup() {
    Serial.begin(115200);
    softwareSerial.begin(9600);

    // Initialize DFPlayer Mini
    if (!player.begin(softwareSerial)) {
        Serial.println("Unable to begin DFPlayer");
        //while (true);
    }
       player.volume(30);
    Serial.println("DFPlayer Mini online.");

    // Set volume
    // Initialize LCD
    lcd.init();
    lcd.backlight();
    pinMode(RELAY_PIN, OUTPUT); // Thiết lập chân GPIO làm output
    digitalWrite(RELAY_PIN, LOW); // Tắt relay ban đầu (LOW)
    distanceQueue = xQueueCreate(5, sizeof(DistanceData));
    xBinarySemaphore = xSemaphoreCreateBinary();
    xMutex = xSemaphoreCreateMutex();

    if (distanceQueue != NULL && xBinarySemaphore != NULL && xMutex != NULL) {
        xTaskCreate(servoControl, "ServoControlTask", 3000, NULL, 1, NULL);
        xTaskCreate(distanceMeasurement, "DistanceMeasureTask", 3000, NULL, 1, NULL);
        xTaskCreate(receiverTask, "ReceiverTask", 3000, NULL, 1, NULL);
        xTaskCreate(checkDistanceAndPlaySound, "checkDistanceAndPlaySound", 3000, NULL, 1, NULL);
     
    } else {
        Serial.println("Failed to create queue or semaphores");
    }
}

void loop() {
    // Empty loop as tasks handle processing
}
