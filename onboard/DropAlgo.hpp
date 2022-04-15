/** \file DropAlgo.hpp
 * @brief All code relating to the drop algoritm
 */

#pragma once

//#include "src/aero-cpp-lib/include/Data.hpp"


/**
 * @brief 
 * @details 
 */
class DropAlgo {
public:
    float time_to_drop = 0, current_down_range, speed;
    /**
     * @brief Construct a new DropAlgo object
     */
    DropAlgo(float _target_lat, float _target_lon) : target_lat(_target_lat), target_lon(_target_lon) {}

    /**
     * @brief 
     * 
     * @return 
     * @return 
     */
    bool update() {
        predicted_drop_time = pow((2*height/GRAVITY), 0.5);
        predicted_down_range = speed*predicted_drop_time;
        
        current_down_range = distance(current_lat, current_lon, target_lat, target_lon);
        last_distance = distance(current_lat, current_lon, last_lat, last_lon);
        speed = last_distance / last_update;
        last_update = millis();
        
        time_to_drop = constrain((predicted_down_range - current_down_range) / speed, 0, 250);
        return true;
    }

    void set_height(float new_height) { height = new_height; height = 100; };
    void set_coords(float lat, float lon) {
      last_lat = current_lat;
      last_lon = current_lon;
      current_lat = lat;
      current_lon = lon;
    };

    void print() {
      char print_buffer[256];
      sprintf(print_buffer, "dy: %-7.2f sp: %-7.2f yt: %-7.2f dx: %-7.2 dr: %-7.2 dt: %-7.2",
              height, speed, predicted_drop_time, predicted_down_range, current_down_range, time_to_drop);
      Serial.println(print_buffer);
    }


private:
    const float GRAVITY = 9.81;
    float height;
    float predicted_drop_time, predicted_down_range, last_distance;
    float current_lat, last_lat, target_lat;
    float current_lon, last_lon, target_lon;
    long last_update = 0;

    float abs_dist(float x1, float y1, float x2, float y2) {
      return sqrt(pow(y2-y1, 2) + pow(x2-x1, 2));
    };

    long double distance(long double lat1, long double long1, long double lat2, long double long2) { 
        lat1 = toRadians(lat1); 
        long1 = toRadians(long1); 
        lat2 = toRadians(lat2); 
        long2 = toRadians(long2); 
          
        long double dlong = long2 - long1; 
        long double dlat = lat2 - lat1; 
      
        long double ans = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlong / 2), 2); 
      
        ans = 2 * asin(sqrt(ans)); 
        
        long double R = 6371000; 
          
        // Calculate the result 
        ans = ans * R; 
      
        return ans; 
    } 

    long double toRadians(const long double degree) { 
        long double one_deg = (PI) / 180; 
        return (one_deg * degree); 
    } 
};
