  #include <ESP8266WiFi.h>
  #include <FirebaseArduino.h>
  #include <SoftwareSerial.h>
  // Set these to run example.
  #define FIREBASE_HOST "thesis-f69f2.firebaseio.com/"
  #define FIREBASE_AUTH "tAX8IXE8OUioiBKkNzRskUTcmfChRZoywsuE9XmF"
  #define WIFI_SSID "ZgAAAFEUC8YAPwHcmehedi"
  #define WIFI_PASSWORD "8D430DDF"
  SoftwareSerial s(D6,D5);
  #include <ArduinoJson.h>
  
  //SimpleTimer timer;
  
  void setup() {
    Serial.begin(115200);
    s.begin(115200);
    while (!Serial) continue;
    delay(1000);
    //timer.setInterval(100L,sendSensor);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to ");
    Serial.print(WIFI_SSID);
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
      delay(500);
    }
    Serial.println();
    Serial.print("Connected to ");
    Serial.println(WIFI_SSID);
      Serial.print("IP Address is : ");
    Serial.println(WiFi.localIP());
    
    Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  //  Serial.print(FIREBASE_HOST +","+ FIREBASE_AUTH);
  }
  //int n = 0;
  //float val;
  void loop() {
    StaticJsonBuffer<1000> jsonBuffer;
    JsonObject& root = jsonBuffer.parseObject(s);
    if (root == JsonObject::invalid())
    {
      return;
    }
    
    Serial.println("JSON received and parsed");
    root.prettyPrintTo(Serial);
    Serial.print("Temperature ");
    int data1=root["t"];
    Serial.println(data1);
    Serial.print("EC ");
    int data2 = root["e"];
    Serial.println(data2);
    Serial.print("TDS ");
    int data3 = root["T"];
    Serial.println(data3);
    Serial.print("DO ");
    int data4 = root["D"];
    Serial.println(data4);
    Serial.print("pH ");
    int data5 = root["p"];
    Serial.println(data5);
    Serial.print("Turbidity Level ");
    int data6 = root["d"];
    Serial.println(data6);
    Serial.print("Turbidity(NTU) ");
    int data7 = root["n"];
    Serial.println(data7);
   
     if (isnan(data1) || isnan(data2) || isnan(data3) || isnan(data4) || isnan(data5) || isnan(data6) || isnan(data7) ) {                                                // Check if any reads failed and exit early (to try again).
      Serial.println(F("Failed to read !"));
      return;
    }
    
    Serial.print("%  Temperature: ");  Serial.print(data1);  Serial.println("Â°C ");
    String fireTemp = String(data1) + String("Â°C");                                                     //convert integer temperature to string temperature
    Serial.print("%  EC: ");  Serial.print(data2);  Serial.println("V ");
    String fireEC = String(data2) + String("V");                                                     //convert integer ec to string ec
    Serial.print("%  TDS: ");  Serial.print(data3);  Serial.println("ppm");
    String fireTds = String(data3) + String("ppm");                                                     //convert integer tds to string tds
    Serial.print("%  DO: ");  Serial.print(data4);  Serial.println("mg\L ");
    String fireDo = String(data4) + String("mg\L");                                                     //conv integer do to string DO
    Serial.print("%  pH ");  Serial.print(data5);  Serial.println("");
    String firepH = String(data5) + String(" ");                                                     //convert integer ph to string ph
    Serial.print("%  Turbidity: ");  Serial.print(data6);  Serial.println("V ");
    String fireTur = String(data6) + String("V");                                                     //convert integer turbidity to string turidity
    Serial.print("%  Turbidity(NTU): ");  Serial.print(data7);  Serial.println("NTU ");
    String fireTurN = String(data7) + String("NTU");                                                     //convert integer turbidity to string turbidity
    delay(4000);
    
    Firebase.pushString("/Thesis/DO", fireDo);                                  //setup path and send readings
    Firebase.pushString("/Thesis/Temperature", fireTemp); //setup path and send readings
    Firebase.pushString("/Thesis/pH", firepH);
    Firebase.pushString("/Thesis/TDS", fireTds);
    Firebase.pushString("/Thesis/Turbidity(V)", fireTur);
    Firebase.pushString("/Thesis/Turbidity(NTU)", fireTurN);
    Firebase.pushString("/Thesis/EC", fireEC);
    
    delay(500);
  }