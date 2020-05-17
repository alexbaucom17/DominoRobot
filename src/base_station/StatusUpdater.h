
#ifndef StatusUpdater_h
#define StatusUpdater_h

#include <Arduino.h> // This has to be before ArduinoJson.h to fix compiler issues
#include <ArduinoJson.h>


class StatusUpdater
{
  public:
    StatusUpdater();

    String getStatusJsonString();

    void updateSensors(bool sensor_1, bool sensor_2, bool sensor_3, bool sensor_4);

    void updateInProgress(bool in_progress);

  private:

    struct Status
    {

      bool sensor_1;
      bool sensor_2;
      bool sensor_3;
      bool sensor_4;
      bool in_progress;
      uint8_t counter; // Just to show that the status is updating. Okay to roll over

      //When adding extra fields, update toJsonString method to serialize and add aditional capacity

      Status():
      sensor_1(false),
      sensor_2(false),
      sensor_3(false),
      sensor_4(false),
      in_progress(false),
      counter(0)
      {
      }

      String toJsonString()
      {
        // Size the object correctly
        const size_t capacity = JSON_OBJECT_SIZE(6); // Update when adding new fields
        DynamicJsonDocument root(capacity);

        // Format to match messages sent by server
        root["type"] = "status";
        JsonObject doc = root.createNestedObject("data");

        // Fill in data
        doc["sensor_1"] = sensor_1;
        doc["sensor_2"] = sensor_2;
        doc["sensor_3"] = sensor_3;
        doc["sensor_4"] = sensor_4;
        doc["in_progress"] = in_progress;
        doc["counter"] = counter++;

        // Serialze and return string
        String msg;
        serializeJson(doc, msg);
        return msg;
      }
    };

    Status currentStatus_;

};

#endif
