
#ifndef StatusUpdater_h
#define StatusUpdater_h

#include <Arduino.h> // This has to be before ArduinoJson.h to fix compiler issues
#include <ArduinoJson.h>
#include <MemoryFree.h>


class StatusUpdater
{
  public:
    StatusUpdater();

    String getStatusJsonString();

    void updatePosition(float x, float y, float a);

    void updateVelocity(float vx, float vy, float va);

    void updateLoopTimes(int controller_loop_ms, int position_loop_ms);

  private:

    struct Status
    {
      // Current position and velocity
      float pos_x;
      float pos_y;
      float pos_a;
      float vel_x;
      float vel_y;
      float vel_a;

      // Loop times
      int controller_loop_ms;
      int position_loop_ms;

      uint8_t counter; // Just to show that the status is updating. Okay to roll over
      int free_memory;

      //When adding extra fields, update toJsonString method to serialize and add aditional capacity

      Status():
      pos_x(0.0),
      pos_y(0.0),
      pos_a(0.0),
      vel_x(0.0),
      vel_y(0.0),
      vel_a(0.0),
      controller_loop_ms(999),
      position_loop_ms(999),
      counter(0),
      free_memory(999)
      {
      }

      String toJsonString()
      {
        // Size the object correctly
        const size_t capacity = JSON_OBJECT_SIZE(12); // Update when adding new fields
        DynamicJsonDocument root(capacity);

        // Format to match messages sent by server
        root["type"] = "status";
        JsonObject doc = root.createNestedObject("data");

        free_memory = freeMemory();

        // Fill in data
        doc["pos_x"] = pos_x;
        doc["pos_y"] = pos_y;
        doc["pos_a"] = pos_a;
        doc["vel_x"] = vel_x;
        doc["vel_y"] = vel_y;
        doc["vel_a"] = vel_a;
        doc["controller_loop_ms"] = controller_loop_ms;
        doc["position_loop_ms"] = position_loop_ms;
        doc["counter"] = counter++;
        doc["free_memory"] = free_memory;

        // Serialze and return string
        String msg;
        serializeJson(doc, msg);
        return msg;
      }
    };

    Status currentStatus_;

};

#endif
