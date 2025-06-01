#ifndef UTILS_H
#define UTILS_H

typedef struct {
  float lat;
  float lon;
} LatLon;

float haversine_distance(float lat1, float lon1, float lat2, float lon2, float diameter);

float angular_diff(float b1, float b2);

float calculate_heading_error(float current_heading, float target_heading);

void swap(LatLon *a, LatLon *b);

float bearing_angle(float lat1, float lon1, float lat2, float lon2);

float random_float(float min, float max);

float clamp(float value);

int next_permutation(int *arr, int n);

#endif