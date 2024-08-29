#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <TimeLib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <WiFi.h>

const char* ssid = "Thien";
const char* password = "123456789";

// Địa chỉ của màn hình LCD qua giao thức I2C
#define LCD_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2

// Định nghĩa chân cảm biến DHT11
#define DHTPIN 4
#define DHTTYPE DHT11

// Khởi tạo đối tượng cảm biến DHT11
DHT dht(DHTPIN, DHTTYPE);

// Chân kết nối của LED
#define LED1_PIN 12
#define LED2_PIN 14
#define LED3_PIN 27
#define LED4_PIN 26

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

int startHour = -1; // Thời gian bắt đầu hẹn giờ LED2
int startMinute = -1;
int endHour = -1;   // Thời gian kết thúc hẹn giờ LED2
int endMinute = -1;
int currentLocalHour;
bool led2On = false;

SemaphoreHandle_t ledSemaphore;
QueueHandle_t sensorQueue;
QueueHandle_t timeQueue;
volatile bool manualMode = true; // Biến trạng thái để xác định chế độ hiện tại (bắt đầu ở chế độ thủ công)

struct SensorData {
    float temperature;
    float humidity;
};

struct TimeData {
    int hour;
    int minute;
    int second;
};

// Hàm để bật LED
void turnOnLED(int pin) {
    digitalWrite(pin, HIGH);
}

// Hàm để tắt LED
void turnOffLED(int pin) {
    digitalWrite(pin, LOW);
}

// Task để điều khiển LED từ Serial Monitor và kiểm tra thời gian hẹn giờ
void controlLEDs(void *pvParameters) {
    (void)pvParameters;

    pinMode(LED2_PIN, OUTPUT);
    digitalWrite(LED2_PIN, LOW);

    while (true) {
        time_t now = time(nullptr);
        int currentHour = hour(now);
        int currentMinute = minute(now);
        currentLocalHour = (currentHour + 7) % 24;  // Điều chỉnh theo giờ địa phương

        if ((currentLocalHour > startHour || (currentLocalHour == startHour && currentMinute >= startMinute)) &&
            (currentLocalHour < endHour || (currentLocalHour == endHour && currentMinute < endMinute))) {
            manualMode = false; // Chuyển sang chế độ tự động khi đến thời gian hẹn giờ
            if (!led2On) {
                if (xSemaphoreTake(ledSemaphore, (TickType_t)10) == pdTRUE) {
                    turnOnLED(LED2_PIN);
                    led2On = true;
                    xSemaphoreGive(ledSemaphore);
                }
            }
        } else {
            if (led2On && !manualMode) {
                if (xSemaphoreTake(ledSemaphore, (TickType_t)10) == pdTRUE) {
                    turnOffLED(LED2_PIN);
                    led2On = false;
                    xSemaphoreGive(ledSemaphore);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Task để lấy thời gian từ máy chủ NTP và gửi dữ liệu thời gian vào queue
void getTimeTask(void *parameter) {
    for (;;) {
        timeClient.update();
        configTime(7 * 3600, 0, "pool.ntp.org"); // UTC+7, múi giờ của Việt Nam

        time_t now = time(nullptr);
        int currentHour = hour(now);
        int currentMinute = minute(now);
        int currentSecond = second(now);

        TimeData timeData = {currentHour, currentMinute, currentSecond};

        if (xQueueSend(timeQueue, &timeData, pdMS_TO_TICKS(10)) != pdPASS) {
            Serial.println("Failed to send time data to the queue");
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Cập nhật thời gian mỗi giây
    }
}

// Task để đo lường và gửi dữ liệu cảm biến vào queue
void sensorDataTask(void *parameter) {
    for (;;) {
        SensorData data;
        data.temperature = dht.readTemperature();
        data.humidity = dht.readHumidity();

        if (xQueueSend(sensorQueue, &data, pdMS_TO_TICKS(10)) != pdPASS) {
            Serial.println("Failed to send sensor data to the queue");
        }

        vTaskDelay(pdMS_TO_TICKS(2000)); // Đo lường mỗi 2 giây
    }
}

// Task để nhận dữ liệu từ queue và hiển thị lên màn hình LCD
void displayTask(void *parameter) {
    for (;;) {
        SensorData sensorData;
        TimeData timeData;

        // Nhận dữ liệu cảm biến từ queue
        if (xQueueReceive(sensorQueue, &sensorData, pdMS_TO_TICKS(10)) == pdPASS) {
            lcd.setCursor(0, 0);
            lcd.print(sensorData.temperature);
            lcd.print("*C");
            lcd.print("   ");
            lcd.print(sensorData.humidity);
            lcd.print("%");
        }

        // Nhận dữ liệu thời gian từ queue
        if (xQueueReceive(timeQueue, &timeData, pdMS_TO_TICKS(10)) == pdPASS) {
            currentLocalHour = (timeData.hour + 7) % 24;  // Điều chỉnh theo giờ địa phương
            lcd.setCursor(0, 1);
            lcd.print("Time: ");
            lcd.print(currentLocalHour);
            lcd.print(":");
            lcd.print(timeData.minute);
            lcd.print(":");
            lcd.print(timeData.second);
        }

        // Chờ 500ms trước khi cập nhật tiếp
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Task để điều khiển LED thủ công bằng cách nhấn 'a' và 'b' và cài đặt thời gian hẹn giờ
void manualControlTask(void *pvParameters) {
    (void)pvParameters;

    pinMode(LED2_PIN, OUTPUT);

    while (true) {
        if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n');
            input.trim(); // Loại bỏ ký tự khoảng trắng ở đầu và cuối chuỗi

            if (input.equals("a")) {
                manualMode = true; // Chuyển sang chế độ thủ công
                if (xSemaphoreTake(ledSemaphore, (TickType_t)10) == pdTRUE) {
                    turnOnLED(LED2_PIN);
                    xSemaphoreGive(ledSemaphore);
                    led2On = true;
                    // Chỉ xóa thời gian hẹn giờ nếu đang trong thời gian hẹn giờ
                    time_t now = time(nullptr);
                    int currentHour = hour(now);
                    int currentMinute = minute(now);
                    currentLocalHour = (currentHour + 7) % 24;  // Điều chỉnh theo giờ địa phương

                    if ((currentLocalHour > startHour || (currentLocalHour == startHour && currentMinute >= startMinute)) &&
                        (currentLocalHour < endHour || (currentLocalHour == endHour && currentMinute < endMinute))) {
                        startHour = -1;
                        startMinute = -1;
                        endHour = -1;
                        endMinute = -1;
                    }
                }
            } else if (input.equals("b")) {
                manualMode = true; // Chuyển sang chế độ thủ công
                if (xSemaphoreTake(ledSemaphore, (TickType_t)10) == pdTRUE) {
                    turnOffLED(LED2_PIN);
                    xSemaphoreGive(ledSemaphore);
                    led2On = false;
                    // Chỉ xóa thời gian hẹn giờ nếu đang trong thời gian hẹn giờ
                    time_t now = time(nullptr);
                    int currentHour = hour(now);
                    int currentMinute = minute(now);
                    currentLocalHour = (currentHour + 7) % 24;  // Điều chỉnh theo giờ địa phương

                    if ((currentLocalHour > startHour || (currentLocalHour == startHour && currentMinute >= startMinute)) &&
                        (currentLocalHour < endHour || (currentLocalHour == endHour && currentMinute < endMinute))) {
                        startHour = -1;
                        startMinute = -1;
                        endHour = -1;
                        endMinute = -1;
                    }
                }
            } else if (input.length() == 11 && input.charAt(2) == ':' && input.charAt(8) == ':') {
                startHour = input.substring(0, 2).toInt();
                startMinute = input.substring(3, 5).toInt();
                endHour = input.substring(6, 8).toInt();
                endMinute = input.substring(9, 11).toInt();
                Serial.print("Thời gian hẹn giờ LED2: ");
                Serial.print(startHour);
                Serial.print(":");
                Serial.print(startMinute);
                Serial.print(" to ");
                Serial.print(endHour);
                Serial.print(":");
                Serial.println(endMinute);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void setup() {
    // Khởi động giao thức I2C
    Wire.begin();

    // Khởi động cảm biến DHT11
    dht.begin();

    // Khởi động màn hình LCD
    lcd.init();
    lcd.backlight();

    // Khởi tạo giao tiếp Serial
    Serial.begin(115200);

    // Kết nối đến mạng WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Đã kết nối đến WiFi");

    // Khởi động kết nối UDP cho NTPClient
    timeClient.begin();
    timeClient.setTimeOffset(25200); // GMT+7 (7*3600)

    // Khởi tạo semaphore
    ledSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(ledSemaphore);

    // Khởi tạo queue
    sensorQueue = xQueueCreate(10, sizeof(SensorData)); // Queue chứa tối đa 10 phần tử SensorData
    timeQueue = xQueueCreate(10, sizeof(TimeData)); // Queue chứa tối đa 10 phần tử TimeData

    // Tạo các task
    xTaskCreate(controlLEDs, "Control LEDs", 2048, NULL, 1, NULL);
    xTaskCreate(getTimeTask, "Get Time", 2048, NULL, 1, NULL);
    xTaskCreate(sensorDataTask, "Sensor Data", 2048, NULL, 1, NULL); // Task gửi dữ liệu cảm biến vào queue
    xTaskCreate(displayTask, "Display Data", 2048, NULL, 1, NULL); // Task nhận dữ liệu từ queue và hiển thị lên LCD
    xTaskCreate(manualControlTask, "Manual Control", 2048, NULL, 1, NULL);
}

void loop() {
    // Vòng lặp chính không làm gì cả vì tất cả đã được xử lý bởi các task FreeRTOS
}
