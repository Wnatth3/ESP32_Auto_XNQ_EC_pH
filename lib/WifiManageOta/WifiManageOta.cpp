#include "Arduino.h"
#include "WifiManageOta.h"

//----------------- WiFi Manager --------------/
// callback notifying us of the need to save config
void saveConfigCallback() {
    deln("Should save config");
    shouldSaveConfig = true;
}

void wifiManagerSetup() {
    // clean FS, for testing
    //  SPIFFS.format();

    // read configuration from FS json
    deln("mounting FS...");

    if (SPIFFS.begin()) {
        deln("mounted file system");
        if (SPIFFS.exists("/config.json")) {
            // file exists, reading and loading
            deln("reading config file");
            File configFile = SPIFFS.open("/config.json", "r");
            if (configFile) {
                deln("opened config file");
                size_t size = configFile.size();
                // Allocate a buffer to store contents of the file.
                std::unique_ptr<char[]> buf(new char[size]);

                configFile.readBytes(buf.get(), size);
                // #if defined(ARDUINOJSON_VERSION_MAJOR) && ARDUINOJSON_VERSION_MAJOR >= 6
                DynamicJsonDocument json(1024);
                auto                deserializeError = deserializeJson(json, buf.get());
                serializeJson(json, Serial);
                if (!deserializeError) {
                    deln("\nparsed json");
                    strcpy(mqtt_server, json["mqtt_server"]);
                    strcpy(mqtt_port, json["mqtt_port"]);
                    strcpy(mqtt_user, json["mqtt_user"]);
                    strcpy(mqtt_pass, json["mqtt_pass"]);
                    if (json["ip"]) {
                        deln("setting custom ip from config");
                        strcpy(static_ip, json["ip"]);
                        strcpy(static_gw, json["gateway"]);
                        strcpy(static_sn, json["subnet"]);
                        strcpy(static_dns, json["dns"]);
                        deln(static_ip);
                    } else {
                        deln("no custom ip in config");
                    }
                } else {
                    deln("failed to load json config");
                }
                // configFile.close();
            }
        }
    } else {
        deln("failed to mount FS");
    }

    wifiManager.setSaveConfigCallback(saveConfigCallback);  // set config save notify callback

    // set static ip
    IPAddress _ip, _gw, _sn, _dns;
    _ip.fromString(static_ip);
    _gw.fromString(static_gw);
    _sn.fromString(static_sn);
    _dns.fromString(static_dns);
    wifiManager.setSTAStaticIPConfig(_ip, _gw, _sn, _dns);

    // add all your parameters here
    wifiManager.addParameter(&custom_mqtt_server);
    wifiManager.addParameter(&custom_mqtt_port);
    wifiManager.addParameter(&custom_mqtt_user);
    wifiManager.addParameter(&custom_mqtt_pass);

    wifiManager.setClass("invert");  // set dark theme

    // wifiManager.resetSettings();  // reset settings - for testing

    wifiManager.setMinimumSignalQuality(30);  // defaults to 8% // set minimu quality of signal so it ignores AP's under that quality

    // wifiManager.setTimeout(120); // auto restart after 30 seconds
    wifiManager.setConfigPortalTimeout(30);  // auto close configportal after 30 seconds

    // fetches ssid and pass and tries to connect
    // if it does not connect it starts an access point with the specified name
    // here  "AutoConnectAP"
    // and goes into a blocking loop awaiting configuration
    if (!wifiManager.autoConnect("AutoConnectAP", "password")) {
        deln("failed to connect and hit timeout");
        // ESP.restart();
    } else {
        deln("connected...yeey :)");
    }

    // read updated parameters
    strcpy(mqtt_server, custom_mqtt_server.getValue());
    strcpy(mqtt_port, custom_mqtt_port.getValue());
    strcpy(mqtt_user, custom_mqtt_user.getValue());
    strcpy(mqtt_pass, custom_mqtt_pass.getValue());
    deln("The values in the file are: ");
    deln("\tmqtt_server : " + String(mqtt_server));
    deln("\tmqtt_port : " + String(mqtt_port));
    deln("\tmqtt_user : " + String(mqtt_user));
    deln("\tmqtt_pass : " + String(mqtt_pass));

    // save the custom parameters to FS
    if (shouldSaveConfig) {
        deln("saving config");
        DynamicJsonDocument json(1024);
        json["mqtt_server"] = mqtt_server;
        json["mqtt_port"]   = mqtt_port;
        json["mqtt_user"]   = mqtt_user;
        json["mqtt_pass"]   = mqtt_pass;
        // Static IP
        json["ip"]      = WiFi.localIP().toString();
        json["gateway"] = WiFi.gatewayIP().toString();
        json["subnet"]  = WiFi.subnetMask().toString();
        json["dns"]     = WiFi.dnsIP().toString();

        File configFile = SPIFFS.open("/config.json", "w");
        if (!configFile) {
            deln("failed to open config file for writing");
        }

        serializeJson(json, Serial);
        serializeJson(json, configFile);
        configFile.close();
        // end save
    }

    deln("local ip: ");
    deln(WiFi.localIP());
    deln(WiFi.gatewayIP());
    deln(WiFi.subnetMask());
    deln(WiFi.dnsIP());

    // if (WiFi.status() == WL_CONNECTED) {
    //     tConnectMqttLoop.enable();
    // }
}

void wifiResetting() {
    if (digitalRead(resetPin) == LOW) {
        delay(50);  // poor mans debounce/press-hold, code not ideal for production
        if (digitalRead(resetPin) == LOW) {
            deln("Reset Button is pressed");
            delay(3000);  // reset delay hold for 3 second
            if (digitalRead(resetPin) == LOW) {
                digitalWrite(ledPin, HIGH);
                deln("Formating SPIFF and Resetting WiFi connection.");
                SPIFFS.format();
                wifiManager.resetSettings();
                // delay(3000);
                ESP.restart();
            }
        }
    }
}

void otaWebUpdateSetup() {
    const char *host = "ESP32";
    /*use mdns for host name resolution*/
    if (!MDNS.begin(host)) {  // http://esp32.local
        Serial.println("Error setting up MDNS responder!");
        while (1) {
            delay(1000);
        }
    }
    Serial.println("mDNS responder started");
    /*return index page which is stored in ud */
    server.on("/", HTTP_GET, []() {
        server.sendHeader("Connection", "close");
        server.send(200, "text/html", loginIndex);
    });
    server.on("/ud", HTTP_GET, []() {
        server.sendHeader("Connection", "close");
        server.send(200, "text/html", ud);
    });
    /*handling uploading firmware file */
    server.on(
        "/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart(); }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    } });
    server.begin();
    Serial.println("\tOTA Web updater setting is done.");
}