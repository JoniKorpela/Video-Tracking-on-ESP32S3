#ifndef TYPE_H
#define TYPE_H
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>
#include <stdbool.h>

#define MAX_FRAME_SIZE 76800 // QVGA

enum
{
    BRIEF_PATCH_SIZE = 49,
    TRACK_BRIEF_SIZE = 128,
    TRACK_BRIEF_BYTES = 16
};

typedef struct
{
    uint16_t x_min;
    uint16_t y_min;
    uint16_t x_max;
    uint16_t y_max;
} target_box;

typedef struct
{
    uint16_t x;
    uint16_t y;
    uint8_t descriptor[TRACK_BRIEF_BYTES];
} feature;

typedef struct
{
    uint32_t count;
    uint32_t size;
    feature *feature_arr;

} feature_vector;

typedef struct
{
    feature feat_1;
    feature feat_2;
    uint16_t distance;
} feature_match;

typedef struct
{
    uint32_t count;
    uint32_t size;
    feature_match *match_arr;
} feature_match_vector;

enum MESSAGE_TYPE
{
    CONTROL_MESSAGE = 1,
    BOX_MESSAGE = 2,
    FRAME_MESSAGE = 3,
    DIM_MESSAGE = 4
};

enum COMMAND_TYPE
{
    STOP_TRACKING = 1,
    DISCONNECT = 2,
};

typedef enum {
    EVENT_FIRST_FRAME = 1 << 0,
    EVENT_NEW_FRAME = 1 << 1,
    EVENT_NEW_BOX = 1 << 2,
    EVENT_CONNECTED = 1 << 3,
    EVENT_START_TRACKING = 1 << 4,
    EVENT_STOP_TRACKING = 1 << 5,
    EVENT_DISCONNECT = 1 << 6
} event;

typedef struct {
    float x;
    float y;
    float vx;
    float vy;
    bool initialized;
} state_vector;

#endif