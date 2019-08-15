#include <Arduino.h>
// #include ""
#include "mavlink_types.h"
#include "common/mavlink.h"
#include "mavlink_conversions.h"
#include "mavlink_helpers.h"
#include "px4_custom_mode.h"
#include "arduino_serial.h"
#include "drone.h"
// #include "mavlink_"
using namespace mavlink_indoor_sdk;

#include <WiFi.h>
#include <PubSubClient.h>

#include <ArduinoJson.h>

WiFiClient wifiClient;
PubSubClient client(wifiClient);

const char *ssid = "........";
const char *password = "........";
const char *mqtt_server = "broker_swarm.local";

Drone *drone;

String mac_addres = "tester";
String navigate_topic = String("drones/") + mac_addres + "/navigate";

/*
{
  "position":{
        "x":0.0,
        "y":0.0,
        "z":0.0
    },
   "velocity":{
        "x":0.0,
        "y":0.0,
        "z":0.0
    },
   "rotation":{
        "roll":0.0,
        "pitch":0.0,
        "yaw":0.0
    },
   "mode":"",
   "armed":true,
   "connected":true

}
*/

void telemetry_to_jsonobject(DynamicJsonDocument &jsonBuffer, Telemetry telemetry)
{

    // JsonObject root = jsonBuffer->as<JsonObject>();

    JsonObject position = jsonBuffer.createNestedObject("position");
    position["x"] = telemetry.position.x;
    position["y"] = telemetry.position.y;
    position["z"] = telemetry.position.z;

    JsonObject velocity = jsonBuffer.createNestedObject("velocity");
    velocity["x"] = telemetry.velocity.x;
    velocity["y"] = telemetry.velocity.y;
    velocity["z"] = telemetry.velocity.z;

    JsonObject rotation = jsonBuffer.createNestedObject("rotation");
    rotation["roll"] = telemetry.rotation.roll;
    rotation["pitch"] = telemetry.rotation.pitch;
    rotation["yaw"] = telemetry.rotation.yaw;

    jsonBuffer["mode"] = mode_ToString(telemetry.mode).c_str();
    jsonBuffer["armed"] = telemetry.armed;
    jsonBuffer["connected"] = telemetry.connected;
    // return root;
}

/*

{
    "position":{
        "x":0.1,
        "y":0.1,
        "z":0.1,
        "yaw":0.1
    },
    "speed":0.5,
    "led":{
        "r":255,
        "g":255,
        "b":255
    }
}

*/

void callback(char *topic, byte *payload, unsigned int length)
{
    if (String(topic) == navigate_topic)
    {
        const size_t capacity = 2 * JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(4) + 50;
        DynamicJsonDocument doc(capacity);
        deserializeJson(doc, payload, length);

        JsonObject position = doc["position"];
        float position_x = position["x"];     // 0.1
        float position_y = position["y"];     // 0.1
        float position_z = position["z"];     // 0.1
        float position_yaw = position["yaw"]; // 0.1

        float speed = doc["speed"];
        // float speed = doc["speed"];

        JsonObject led = doc["led"];
        int led_r = led["r"]; // 255
        int led_g = led["g"]; // 255
        int led_b = led["b"]; // 255

        // drone->navigate({position_x, position_y, position_z, position_yaw}, FRAME_LOCAL, speed);
        drone->set_position({position_x, position_y, position_z, position_yaw}, FRAME_LOCAL);
    }
}

void setup()
{
    Serial.begin(115200);
    Serial1.begin(57600);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    mac_addres = WiFi.macAddress();

    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);

    ArduinoSerial as(&Serial1);
    as.start();
    autopilot_interface::AutopilotInterface ai(&as);
    // ai.start();
    drone = new Drone(&ai);
    drone->start();

    navigate_topic = String("drones/") + mac_addres + "/navigate";
    client.subscribe(navigate_topic.c_str());
    // drone->arm();
    // drone->sleep(1000);
    // drone->set_position({0, 0, 2, 0}, FRAME_LOCAL);
    // // drone->sleep(10000);
    // for (size_t i = 0; i < 50; i++)
    // {
    //     Serial.println(drone->get_telemetry(FRAME_LOCAL).ToString());
    //     const size_t capacity = 3 * JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(6);
    //     DynamicJsonDocument jsonBuffer(capacity);
    //     telemetry_to_jsonobject(jsonBuffer, drone->get_telemetry(FRAME_LOCAL));
    //     String json_p("");
    //     serializeJsonPretty(jsonBuffer, json_p);
    //     client.publish("telemetry", json_p.c_str());
    //     drone->sleep(100);
    // }

    // drone->land();
    // drone->sleep(4000);
    // drone->disarm();

    // mavlink_message_t msg;
    // mavlink_heartbeat_t hrbt;
    // mavlink_msg_heartbeat_decode(&msg, &hrbt);
}
unsigned long prev_time = 0;

void publish_telemetry()
{
    const size_t capacity = 3 * JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(6);
    DynamicJsonDocument jsonBuffer(capacity);

    telemetry_to_jsonobject(jsonBuffer, drone->get_telemetry(FRAME_LOCAL));

    String json_p("");
    serializeJsonPretty(jsonBuffer, json_p);
    Serial.println(json_p);

    String telemetry_topic = String("drones/") + mac_addres + "/telemetry";
    client.publish(telemetry_topic.c_str(), json_p.c_str());
}

void loop()
{
    if ((millis() - prev_time) >= 200)
    {
        // Serial.println(drone->get_telemetry(FRAME_LOCAL).ToString());
        publish_telemetry();
        prev_time = millis();
    }
    client.loop();
}