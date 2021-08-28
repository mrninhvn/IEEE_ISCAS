#include <WiFi.h>
#include <FirebaseESP32.h>


#define FIREBASE_HOST "kien31051999-firebase.firebaseio.com" //Do not include https:// in FIREBASE_HOST
#define FIREBASE_AUTH "Zp2lZjOs8F1KKOzc1M04yvuTaAxIYeSoXUS72ZOD"
#define WIFI_SSID "Huy"
#define WIFI_PASSWORD "08031973"


//Define FirebaseESP32 data object
FirebaseData firebaseData;

FirebaseJson json;

String path = "/Spoon-Timestamp";

String value;
String fx;
String fy;
String fz;
long lastTime = 0;

#define RXD2 16
#define TXD2 17

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  pinMode(2, OUTPUT);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);

  //Set database read timeout to 1 minute (max 15 minutes)
  Firebase.setReadTimeout(firebaseData, 1000 * 60);
  //tiny, small, medium, large and unlimited.
  //Size and its write timeout e.g. tiny (1s), small (10s), medium (30s) and large (60s).
  Firebase.setwriteSizeLimit(firebaseData, "tiny");

  Serial.println("-------START-------");
}

void loop() {
 if (Serial2.available()) {
  value = Serial2.readStringUntil('\n');
 }

 if (value.length() == 26)
 {

  fx = "";
  fx = fx + value[4] + value[5] + value[6] + value[7];

  fy = "";
  fy = fy + value[12] + value[13] + value[14] + value[15];

  fz = "";
  fz = fz + value[20] + value[21] + value[22] + value[23];

  double x = fx.toFloat();
  double y = fy.toFloat();
  double z = fz.toFloat();

  Serial.print("fx: "); Serial.println(x);
  Serial.print("fy: "); Serial.println(y);
  Serial.print("fz: "); Serial.println(z);

  json.clear().add("fx", x);
  json.add("fy", y);
  json.add("fz", z);
  if (Firebase.pushJSON(firebaseData, path, json))
  {
    digitalWrite(2, HIGH);
    Serial.println("PASSED");
  }
  else
  {
    Serial.println("FAILED");
    Serial.println("REASON: " + firebaseData.errorReason());
    Serial.println("------------------------------------");
    Serial.println();
  }
  
  value = "";
  lastTime = millis();
  digitalWrite(2, LOW);

 }

}
