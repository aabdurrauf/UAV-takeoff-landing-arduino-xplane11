#include <math.h>
#include <stdlib.h>
#include "utils.h"

#define DEG_TO_RAD 0.017453292519943295f
#define RAD_TO_DEG 57.29577951308232f

// Haversine distance in meters
float haversine_distance(float lat1, float lon1, float lat2, float lon2, float diameter) {
  float dLat = (lat2 - lat1) * DEG_TO_RAD;
  float dLon = (lon2 - lon1) * DEG_TO_RAD;

  lat1 *= DEG_TO_RAD;
  lat2 *= DEG_TO_RAD;

  float a = sin(dLat / 2.0f) * sin(dLat / 2.0f) +
            cos(lat1) * cos(lat2) * sin(dLon / 2.0f) * sin(dLon / 2.0f);
  float c = 2.0f * atan2(sqrt(a), sqrt(1.0f - a));
  return diameter * c;
}

// Returns the smallest absolute difference between two angles (0–180 degrees)
float angular_diff(float b1, float b2) {
    float diff = fabsf(b1 - b2);
    return diff < (360.0f - diff) ? diff : (360.0f - diff);
}

// Returns signed heading error in range [-180, 180]
float calculate_heading_error(float current_heading, float target_heading) {
    float error = fmodf((target_heading - current_heading + 360.0f), 360.0f);
    if (error > 180.0f) {
        error -= 360.0f;
    }
    return error;
}

void swap(LatLon *a, LatLon *b) {
  LatLon temp = *a;
  *a = *b;
  *b = temp;
}

float bearing_angle(float lat1, float lon1, float lat2, float lon2) {
    float lat1_rad = lat1 * DEG_TO_RAD;
    float lat2_rad = lat2 * DEG_TO_RAD;
    float dlon_rad = (lon2 - lon1) * DEG_TO_RAD;

    float x = sinf(dlon_rad) * cosf(lat2_rad);
    float y = cosf(lat1_rad) * sinf(lat2_rad) -
              sinf(lat1_rad) * cosf(lat2_rad) * cosf(dlon_rad);

    float initial_bearing = atan2f(x, y);
    float bearing_deg = fmodf((initial_bearing * RAD_TO_DEG) + 360.0f, 360.0f);
    return bearing_deg;
}

// Generate random float between min and max
float random_float(float min, float max) {
  return min + ((float)rand() / (float)RAND_MAX) * (max - min);
}

// clamp value between -1 and 1
float clamp(float value) {
  return fmax(fmin(value, 1.0f), -1.0f);
}

// Returns 1 if next permutation exists, 0 if wrapped around
int next_permutation(int *arr, int n) {
  int i = n - 2;
  while (i >= 0 && arr[i] >= arr[i + 1]) i--;
  if (i < 0) return 0;

  int j = n - 1;
  while (arr[j] <= arr[i]) j--;

  int temp = arr[i]; arr[i] = arr[j]; arr[j] = temp;

  for (int a = i + 1, b = n - 1; a < b; a++, b--) {
    temp = arr[a]; arr[a] = arr[b]; arr[b] = temp;
  }

  return 1;
}
