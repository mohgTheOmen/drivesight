#include <WiFi.h>
#include "esp_camera.h"
#include <Bluepad32.h>
#include "secret.h"  // Contains WiFi credentials (WIFI_SSID and WIFI_PASSWORD)

#define LED_PIN 2  // GPIO2 as status LED

ControllerPtr myController = nullptr; // Store a single connected controller
String picoData = "<b>PLACEHOLDER</b>"; // Placeholder for data exchange with Pico

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// Camera GPIO pin definitions (specific to the ESP32-S3 WROOM with OV3660)
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM  4
#define SIOC_GPIO_NUM  5
#define Y2_GPIO_NUM    11
#define Y3_GPIO_NUM    9
#define Y4_GPIO_NUM    8
#define Y5_GPIO_NUM    10
#define Y6_GPIO_NUM    12
#define Y7_GPIO_NUM    18
#define Y8_GPIO_NUM    17
#define Y9_GPIO_NUM    16
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  7
#define PCLK_GPIO_NUM  13

// Forward declarations for FreeRTOS tasks
void wifiTask(void *pvParameters);
void controllerTask(void *pvParameters);
void cameraServerTask(void *pvParameters);

// Web server instance
WiFiServer server(80);

// Camera setup function
void setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  // Data pins
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  // Control pins
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  // Clock and image format
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count     = 1;
  config.fb_location  = CAMERA_FB_IN_PSRAM;

  // Adjust settings based on PSRAM availability
  if (psramFound()) {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    Serial.println("Found PSRAM, stream settings adjusted accordingly.");
  } else {
    config.frame_size = FRAMESIZE_QQVGA;
    config.jpeg_quality = 15;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  // Initialize the camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    while (true);  // Halt system if camera fails to initialize
  }
}

// Called when a gamepad connects
void onConnectedController(ControllerPtr ctl) {
  if (myController == nullptr) {
    Serial.printf("Controller connected\n");
    myController = ctl;
  } else {
    Serial.println("Controller already connected");
  }
}

// Called when a gamepad disconnects
void onDisconnectedController(ControllerPtr ctl) {
  if (myController == ctl) {
    Serial.println("Controller disconnected");
    myController = nullptr;
  }
}

// Format and send controller data to Pico over UART
void dumpGamepad(ControllerPtr ctl) {
  String data = String(ctl->index()) + "," + String(ctl->dpad(), HEX) + "," +
                String(ctl->buttons(), HEX) + "," + String(ctl->axisX()) + "," +
                String(ctl->axisY()) + "," + String(ctl->axisRX()) + "," +
                String(ctl->axisRY()) + "," + String(ctl->brake()) + "," +
                String(ctl->throttle()) + "," + String(ctl->miscButtons(), HEX);

  Serial2.println(data);  // Send to Pico
  Serial.println("Sent controller data: " + data);  // Debug print
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 19, 20); // UART2 for communication with Pico
  Serial.setDebugOutput(true);
  pinMode(LED_PIN, OUTPUT);

  // Initialize Bluepad32 and set callbacks
  Serial.printf("Bluepad32 firmware: %s\n", BP32.firmwareVersion());
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();  // Optional: clear previous pairings

  setupCamera();  // Initialize camera

  // Wi-Fi setup in station mode
  WiFi.mode(WIFI_STA);
  delay(100);
  server.begin();  // Start web server

  // Start FreeRTOS tasks on different cores
  xTaskCreatePinnedToCore(wifiTask, "WiFiTask", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(controllerTask, "ControllerTask", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(cameraServerTask, "CameraTask", 8192, NULL, 1, NULL, 1);
}

void loop() {
  delay(100);  // Loop does nothing; all logic handled in tasks
}

// Wi-Fi reconnect logic runs in its own task
void wifiTask(void *pvParameters) {
  while (true) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi not connected. Attempting to connect...");
      WiFi.disconnect();
      WiFi.begin(ssid, password);

      unsigned long startAttemptTime = millis();
      const unsigned long timeout = 10000;

      while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < timeout) {
        Serial.print(".");
        vTaskDelay(pdMS_TO_TICKS(500));
      }

      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
      } else {
        Serial.println("\nWiFi connection failed. Retrying in 5s...");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(5000));  // Check every 5 seconds
  }
}

// Controller data polling and transmission task
void controllerTask(void *pvParameters) {
  while (true) {
    bool dataUpdated = BP32.update();
    if (dataUpdated && myController != nullptr) {
      dumpGamepad(myController);
    }
    vTaskDelay(pdMS_TO_TICKS(100));  // 10 updates per second
  }
}

// Camera MJPEG streaming server task
void cameraServerTask(void *pvParameters) {
  WiFiClient client;

  while (true) {
    // Wait for a new client connection
    if (!client || !client.connected()) {
      client = server.available();
      if (client) {
        Serial.println("Client Connected");
        digitalWrite(LED_PIN, HIGH);

        // Send MJPEG stream header
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
        client.println();
      }
    }

    // If a client is connected, stream camera frames
    if (client && client.connected()) {
      camera_fb_t *fb = esp_camera_fb_get();
      if (!fb) {
        Serial.println("Camera capture failed");
        client.stop();
        digitalWrite(LED_PIN, LOW);
        continue;
      }

      // Send one JPEG frame
      client.println("--frame");
      client.println("Content-Type: image/jpeg");
      client.printf("Content-Length: %d\r\n\r\n", fb->len);
      client.write(fb->buf, fb->len);
      client.println();

      esp_camera_fb_return(fb);
      vTaskDelay(pdMS_TO_TICKS(50));  // Cap at ~20 FPS
    } else {
      // No client, turn off LED
      digitalWrite(LED_PIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}
