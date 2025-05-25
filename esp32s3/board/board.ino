#include <WiFi.h>
#include "esp_camera.h"

#include <Bluepad32.h>

#include "secret.h"

#define LED_PIN 2

ControllerPtr myController = nullptr; // Only one controller
String picoData = "<b>PLACEHOLDER</b>";

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// Set up camera pins (change if needed)
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM  4
#define SIOC_GPIO_NUM  5

#define Y2_GPIO_NUM 11
#define Y3_GPIO_NUM 9
#define Y4_GPIO_NUM 8
#define Y5_GPIO_NUM 10
#define Y6_GPIO_NUM 12
#define Y7_GPIO_NUM 18
#define Y8_GPIO_NUM 17
#define Y9_GPIO_NUM 16

#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  7
#define PCLK_GPIO_NUM  13

//void readUARTTask(void *pvParameters);
void wifiTask(void *pvParameters);
void controllerTask(void *pvParameters);
void cameraServerTask(void *pvParameters);

// Web server on port 80
WiFiServer server(80);

// Camera setup
void setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count     = 1;
  config.fb_location  = CAMERA_FB_IN_PSRAM;

  bool has_psram = psramFound();
  if (has_psram) {
    config.frame_size = FRAMESIZE_QVGA;  // higher if you want
    config.jpeg_quality = 12;
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    Serial.println("Found PSRAM, stream settings adjusted accordingly.");
  } else {
    config.frame_size = FRAMESIZE_QQVGA;  // smaller frames
    config.jpeg_quality = 15;            // lower quality
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  // Initialize camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    while (true);
  }
}

//int k = 0;
//// Function to read data from UART
//void readUARTData() {
////  if (Serial2.available()) {
////    picoData = Serial2.readStringUntil('\n');  // Read data sent by Pico
////  }
//  // Simulate receiving data from Pico
//  static unsigned long lastMillis = 0;
//  unsigned long currentMillis = millis();
//
//  // Simulate incoming UART data every 2 seconds
//  if (currentMillis - lastMillis >= 2000) {
//    picoData = "";  // Clear after processing
//    picoData = "Test data from Pico - ";  // Simulated data
//    picoData += k;
//    picoData += "\n";
//    lastMillis = currentMillis;  // Reset the timer
//    Serial.println("Simulated picoData: " + picoData);  // Print the simulated data
//    k++;
//  }
//}

// This callback gets called when a gamepad is connected
void onConnectedController(ControllerPtr ctl) {
    if (myController == nullptr) {
        Serial.printf("Controller connected\n");
        myController = ctl;
    } else {
        Serial.println("Controller already connected");
    }
}

// This callback gets called when a gamepad is disconnected
void onDisconnectedController(ControllerPtr ctl) {
    if (myController == ctl) {
        Serial.println("Controller disconnected");
        myController = nullptr;
    }
}

// Function to dump gamepad data
void dumpGamepad(ControllerPtr ctl) {
    String data = String(ctl->index()) + "," + String(ctl->dpad(), HEX) + "," + 
                  String(ctl->buttons(), HEX) + "," + String(ctl->axisX()) + "," + 
                  String(ctl->axisY()) + "," + String(ctl->axisRX()) + "," + 
                  String(ctl->axisRY()) + "," + String(ctl->brake()) + "," + 
                  String(ctl->throttle()) + "," + String(ctl->miscButtons(), HEX);

    // Send the data over UART2
    Serial2.println(data);
    Serial.println("Sent controller data: " + data);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 19, 20);
  Serial.setDebugOutput(true);
  pinMode(LED_PIN, OUTPUT);
  Serial.printf("Bluepad32 firmware: %s\n", BP32.firmwareVersion());
  
  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);
  
  // Forget any paired Bluetooth devices (optional)
  BP32.forgetBluetoothKeys();
  Serial.println();

  setupCamera();

  WiFi.mode(WIFI_STA);
  delay(100);

  server.begin();

  xTaskCreatePinnedToCore(wifiTask, "WiFiTask", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(controllerTask, "ControllerTask", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(cameraServerTask, "CameraTask", 8192, NULL, 1, NULL, 1);
//  xTaskCreatePinnedToCore(readUARTTask, "ReadUARTTask", 2048, NULL, 1, NULL, 0);
}

void loop() {
    delay(100); // loop does nothing now
}

//void readUARTTask(void *pvParameters) {
//  while (true) {
//    readUARTData();  // Read and process data from UART
//    vTaskDelay(pdMS_TO_TICKS(10));  // Yield to other tasks
//  }
//}

// Wi-Fi Connection Task
void wifiTask(void *pvParameters) {
  //  // Set up the ESP32 as an Access Point
  //  if (!WiFi.softAP(ssid, password)) {
  //    log_e("Soft AP creation failed.");
  //    while(1);
  //  }
  //  Serial.print("Setting AP... ");
  //  Serial.print(ssid);
  //  Serial.print(" IP address: ");
  //  Serial.println(WiFi.softAPIP());
  
    while (true) {
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi not connected. Attempting to connect...");
            WiFi.disconnect();
            WiFi.begin(ssid, password);

            unsigned long startAttemptTime = millis();
            const unsigned long timeout = 10000; // 10 seconds max wait

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

        // Check every 5 seconds for disconnect
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}


// Controller Task
void controllerTask(void *pvParameters) {
    while (true) {
        bool dataUpdated = BP32.update();
        if (dataUpdated && myController != nullptr) {
            dumpGamepad(myController);
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // 100ms
    }
}

// Camera Server Task
//void cameraServerTask(void *pvParameters) {
//    WiFiClient client;
//
//    while (true) {
//        client = server.available();
//
//        if (client) {
//            Serial.println("Client Connected");
//            digitalWrite(LED_PIN, HIGH);
//
//            String request = client.readStringUntil('\r');
//            client.flush();
//
//            if (request.indexOf("GET / ") != -1) {
//                // Serve HTML Page
//                client.println("HTTP/1.1 200 OK");
//                client.println("Content-Type: text/html");
//                client.println("Connection: close");
//                client.println();
//                client.println("<!DOCTYPE html><html><head><title>ESP32S3 Camera</title></head><body>");
//                client.println("<h1>ESP32S3 Camera Snapshot</h1>");
//                client.println("<img id=\"cameraStream\" width=\"640\" height=\"480\">");
//                client.println("<p id=\"picoData\">Loading...</p>");
//                client.println("<script>");
//                client.println("function fetchFrame() {");
//                client.println("  document.getElementById('cameraStream').src = '/frame?' + new Date().getTime();");
//                client.println("}");
//                client.println("setInterval(fetchFrame, 50);"); // Fetch new frame every 100ms (10FPS)");
//
//                client.println("function fetchPicoData() {");
//                client.println("  fetch('/getPicoData')");
//                client.println("    .then(response => response.text())");
//                client.println("    .then(data => {");
//                client.println("      document.getElementById('picoData').innerText = 'Data from Pico: ' + data;");
//                client.println("    });");
//                client.println("}");
//                client.println("setInterval(fetchPicoData, 1000);"); // Fetch Pico data every 1 second
//                client.println("</script>");
//                client.println("</body></html>");
//                client.stop();
//                digitalWrite(LED_PIN, LOW);
//
//            } else if (request.indexOf("GET /frame") != -1) {
//                // Serve a single camera frame
//                camera_fb_t *fb = esp_camera_fb_get();
//                if (!fb) {
//                    Serial.println("Camera capture failed");
//                    client.stop();
//                    digitalWrite(LED_PIN, LOW);
//                    continue;
//                }
//
//                client.println("HTTP/1.1 200 OK");
//                client.println("Content-Type: image/jpeg");
//                client.println("Content-Length: " + String(fb->len));
//                client.println("Connection: close");
//                client.println();
//                client.write(fb->buf, fb->len);
//                esp_camera_fb_return(fb);
//
//                client.stop();
//                digitalWrite(LED_PIN, LOW);
//
//            } else if (request.indexOf("GET /getPicoData") != -1) {
//                // Serve Pico Data
//                client.println("HTTP/1.1 200 OK");
//                client.println("Content-Type: text/plain");
//                client.println("Connection: close");
//                client.println();
//                client.println(picoData);
//                client.stop();
//                digitalWrite(LED_PIN, LOW);
//
//            } else {
//                // Unknown request
//                client.println("HTTP/1.1 404 Not Found");
//                client.println("Content-Type: text/plain");
//                client.println("Connection: close");
//                client.println();
//                client.println("404 Not Found");
//                client.stop();
//                digitalWrite(LED_PIN, LOW);
//            }
//        }
//
//        vTaskDelay(pdMS_TO_TICKS(10));
//    }
//}

//void cameraServerTask(void *pvParameters) {
//    while (true) {
//        WiFiClient client = server.available();
//        if (client) {
//            if (client.available()) {
//                String request = client.readStringUntil('\r');
//                client.flush();
//
//                if (request.indexOf("/stream") != -1) {
//                    Serial.println("Stream requested");
//
//                    digitalWrite(LED_PIN, HIGH);
//
//                    // Send MJPEG headers
//                    client.println("HTTP/1.1 200 OK");
//                    client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
//                    client.println();
//
//                    while (client.connected()) {
//                        camera_fb_t * fb = esp_camera_fb_get();
//                        if (!fb) {
//                            Serial.println("Camera capture failed");
//                            break;
//                        }
//
//                        client.println("--frame");
//                        client.println("Content-Type: image/jpeg");
//                        client.printf("Content-Length: %d\r\n\r\n", fb->len);
//                        client.write(fb->buf, fb->len);
//                        client.println();
//                        esp_camera_fb_return(fb);
//
//                        vTaskDelay(pdMS_TO_TICKS(50)); // Adjust frame rate
//                    }
//
//                    digitalWrite(LED_PIN, LOW);
//                } else {
//                    client.println("HTTP/1.1 200 OK");
//                    client.println("Content-Type: text/html");
//                    client.println();
//                    client.println("<html><body><h1>ESP32S3 Camera</h1>");
//                    client.println("<img src=\"/stream\" width=\"640\" height=\"480\">");
//                    client.println("</body></html>");
//                }
//
//                client.stop();
//            }
//        }
//        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to yield
//    }
//}

void cameraServerTask(void *pvParameters) {
    WiFiClient client;

    while (true) {
        if (!client || !client.connected()) {
            client = server.available();
            if (client) {
                Serial.println("Client Connected");
                digitalWrite(LED_PIN, HIGH);

                // Send HTTP headers for MJPEG stream
                client.println("HTTP/1.1 200 OK");
                client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
                client.println();
            }
        }

        if (client && client.connected()) {
            camera_fb_t *fb = esp_camera_fb_get();
            if (!fb) {
                Serial.println("Camera capture failed");
                client.stop();
                digitalWrite(LED_PIN, LOW);
                continue;
            }

            // Send a single JPEG frame
            client.println("--frame");
            client.println("Content-Type: image/jpeg");
            client.printf("Content-Length: %d\r\n\r\n", fb->len);
            client.write(fb->buf, fb->len);
            client.println();

            esp_camera_fb_return(fb);

            // Limit framerate to ~20 FPS
            vTaskDelay(pdMS_TO_TICKS(50));
        } else {
            // No client connected
            digitalWrite(LED_PIN, LOW);
            vTaskDelay(pdMS_TO_TICKS(10)); // Sleep briefly
        }
    }
}
