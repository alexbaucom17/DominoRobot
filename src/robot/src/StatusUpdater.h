
#ifndef StatusUpdater_h
#define StatusUpdater_h

#include <ArduinoJson/ArduinoJson.h>


class StatusUpdater
{
  public:
    StatusUpdater();

    std::string getStatusJsonString();

    void updatePosition(float x, float y, float a);

    void updateVelocity(float vx, float vy, float va);

    void updatePositionConfidence(float cx, float cy, float ca);

    void updateLoopTimes(int controller_loop_ms, int position_loop_ms);

    void updateInProgress(bool in_progress);

    bool getInProgress() { return currentStatus_.in_progress; };

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
      uint8_t confidence_x;
      uint8_t confidence_y;
      uint8_t confidence_a;

      // Loop times
      int controller_loop_ms;
      int position_loop_ms;

      bool in_progress;
      uint8_t counter; // Just to show that the status is updating. Okay to roll over

      //When adding extra fields, update toJsonString method to serialize and add aditional capacity

      Status():
      pos_x(0.0),
      pos_y(0.0),
      pos_a(0.0),
      vel_x(0.0),
      vel_y(0.0),
      vel_a(0.0),
      confidence_x(0),
      confidence_y(0),
      confidence_a(0),
      controller_loop_ms(999),
      position_loop_ms(999),
      in_progress(false),
      counter(0)
      {
      }

      std::string toJsonString()
      {
        // Size the object correctly
        const size_t capacity = JSON_OBJECT_SIZE(16); // Update when adding new fields
        DynamicJsonDocument root(capacity);

        // Format to match messages sent by server
        root["type"] = "status";
        JsonObject doc = root.createNestedObject("data");

        // Fill in data
        doc["pos_x"] = pos_x;
        doc["pos_y"] = pos_y;
        doc["pos_a"] = pos_a;
        doc["vel_x"] = vel_x;
        doc["vel_y"] = vel_y;
        doc["vel_a"] = vel_a;
        doc["confidence_x"] = confidence_x;
        doc["confidence_y"] = confidence_y;
        doc["confidence_a"] = confidence_a;
        doc["controller_loop_ms"] = controller_loop_ms;
        doc["position_loop_ms"] = position_loop_ms;
        doc["in_progress"] = in_progress;
        doc["counter"] = counter++;

        // Serialze and return string
        std::string msg;
        serializeJson(doc, msg);
        return msg;
      }
    };

    Status currentStatus_;

};

#endif