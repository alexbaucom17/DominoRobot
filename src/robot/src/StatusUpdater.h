
#ifndef StatusUpdater_h
#define StatusUpdater_h

#include <ArduinoJson/ArduinoJson.h>
#include "utils.h"

class StatusUpdater
{
  public:
    StatusUpdater();

    std::string getStatusJsonString();

    void updatePosition(float x, float y, float a);

    void updateVelocity(float vx, float vy, float va);

    void updateControlLoopTime(int controller_loop_ms);

    void updatePositionLoopTime(int position_loop_ms);

    void updateInProgress(bool in_progress);

    bool getInProgress() const { return currentStatus_.in_progress; };
    
    void setErrorStatus() { currentStatus_.error_status = true;}

    void clearErrorStatus() {currentStatus_.error_status = false;};

    bool getErrorStatus() const {return currentStatus_.error_status;}

    void updateLocalizationMetrics(LocalizationMetrics localization_metrics);

    float getLocalizationConfidence() const {return currentStatus_.localization_metrics.total_confidence;};

    void update_motor_driver_connected(bool connected);

    void update_lifter_driver_connected(bool connected);

    void updateCameraDebug(CameraDebug camera_debug) {currentStatus_.camera_debug = camera_debug;};

    void updateVisionControllerPose(Point pose);

    struct Status
    {
      // Current position and velocity
      float pos_x;
      float pos_y;
      float pos_a;
      float vel_x;
      float vel_y;
      float vel_a;

      // Vision tracker pose
      float vision_x;
      float vision_y;
      float vision_a;

      // Loop times
      int controller_loop_ms;
      int position_loop_ms;

      bool in_progress;
      bool error_status;
      uint8_t counter; // Just to show that the status is updating. Okay to roll over

      bool motor_driver_connected;
      bool lifter_driver_connected;

      LocalizationMetrics localization_metrics;
      CameraDebug camera_debug;

      //When adding extra fields, update toJsonString method to serialize and add additional capacity

      Status():
      pos_x(0.0),
      pos_y(0.0),
      pos_a(0.0),
      vel_x(0.0),
      vel_y(0.0),
      vel_a(0.0),
      vision_x(0.0),
      vision_y(0.0),
      vision_a(0.0),
      controller_loop_ms(999),
      position_loop_ms(999),
      in_progress(false),
      error_status(false),
      counter(0),
      motor_driver_connected(false),
      lifter_driver_connected(false),
      localization_metrics(),
      camera_debug()
      {
      }

      std::string toJsonString()
      {
        // Size the object correctly
        const size_t capacity = JSON_OBJECT_SIZE(50); // Update when adding new fields
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
        doc["controller_loop_ms"] = controller_loop_ms;
        doc["position_loop_ms"] = position_loop_ms;
        doc["in_progress"] = in_progress;
        doc["error_status"] = error_status;
        doc["counter"] = counter++;
        doc["motor_driver_connected"] = motor_driver_connected;
        doc["lifter_driver_connected"] = lifter_driver_connected;
        doc["localization_confidence_x"] = localization_metrics.confidence_x;
        doc["localization_confidence_y"] = localization_metrics.confidence_y;
        doc["localization_confidence_a"] = localization_metrics.confidence_a;
        doc["localization_total_confidence"] = localization_metrics.total_confidence;
        doc["last_position_uncertainty"] = localization_metrics.last_position_uncertainty;
        doc["cam_side_ok"] = camera_debug.side_ok;
        doc["cam_rear_ok"] = camera_debug.rear_ok;
        doc["cam_both_ok"] = camera_debug.both_ok;
        doc["cam_side_u"] = camera_debug.side_u;
        doc["cam_side_v"] = camera_debug.side_v;
        doc["cam_rear_u"] = camera_debug.rear_u;
        doc["cam_rear_v"] = camera_debug.rear_v;
        doc["cam_side_x"] = camera_debug.side_x;
        doc["cam_side_y"] = camera_debug.side_y;
        doc["cam_rear_x"] = camera_debug.rear_x;
        doc["cam_rear_y"] = camera_debug.rear_y;
        doc["cam_pose_x"] = camera_debug.pose_x;
        doc["cam_pose_y"] = camera_debug.pose_y;
        doc["cam_pose_a"] = camera_debug.pose_a;
        doc["cam_loop_ms"] = camera_debug.loop_ms;
        doc["vision_x"] = vision_x;
        doc["vision_y"] = vision_y;
        doc["vision_a"] = vision_a;

        // Serialize and return string
        std::string msg;
        serializeJson(root, msg);
        return msg;
      }
    };

    Status getStatus() { return currentStatus_;};

  private:
    Status currentStatus_;

};

#endif
