
#ifndef StatusUpdater_h
#define StatusUpdater_h

#include <Arduino.h> // This has to be before ArduinoJson.h to fix compiler issues
#include <ArduinoSTL.h>
#include <ArduinoJson.h>


class StatusUpdater
{
  public:
    StatusUpdater();

    String getStatusJsonString() const;

    void updatePosition(float x, float y, float a);

    void updateVelocity(float vx, float vy, float va);

    void updateFrequencies(float controller_freq, float position_freq);

    void update_task(String cur_task);

    void addNote(String note);

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
      float controller_freq;
      float position_freq;

      String current_task;
      std::vector<String> notes;

      //When adding extra fields, update toJsonString method to serialize and add aditional capacity

      Status():
      pos_x(0.0),
      pos_y(0.0),
      pos_a(0.0),
      vel_x(0.0),
      vel_y(0.0),
      vel_a(0.0),
      controller_freq(0.0),
      position_freq(0.0),
      current_task("NONE"),
      notes({})        
      {
      }

      String toJsonString()
      {
        // Size the object correctly
        int array_size = notes.size();
        const size_t capacity = JSON_ARRAY_SIZE(array_size) + JSON_OBJECT_SIZE(12); 
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
        doc["controller_freq"] = controller_freq;
        doc["position_freq"] = position_freq;
        doc["current_task"] = current_task;

        // Fill in variable size string data
        JsonArray notes_doc = doc.createNestedArray("notes");
        for (int i = 0; i < array_size; i++)
        {
          notes_doc.add(notes[i]);
        }
        notes.clear(); // Clear out old notes TODO: Make notes last for some amount of time

        // Serialze and return string
        String msg;
        serializeJson(doc, msg);
        return msg;
      }
    };

    Status currentStatus_;

};

#endif