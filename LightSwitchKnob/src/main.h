#include "proto_gen/smartknob.pb.h"
#define PI 3.1415926535897932384626433832795
enum class CommandType
{
    CALIBRATE,
    CONFIG,
    HAPTIC,
};

struct HapticData
{
    bool press;
};

struct Command
{
    CommandType command_type;
    union CommandData
    {
        uint8_t unused;
        PB_SmartKnobConfig config;
        HapticData haptic;
    };
    CommandData data;
};

static PB_SmartKnobConfig configs[] = {
    // int32_t position;
    // float sub_position_unit;
    // uint8_t position_nonce;
    // int32_t min_position;
    // int32_t max_position;
    // float position_width_radians;
    // float detent_strength_unit;
    // float endstop_strength_unit;
    // float snap_point;
    // char text[51];
    // pb_size_t detent_positions_count;
    // int32_t detent_positions[5];
    // float snap_point_bias;
    // int8_t led_hue;

    {
        0,
        0,
        3,
        0,
        1,
        60 * PI / 180,
        1,
        1,
        0.55, // Note the snap point is slightly past the midpoint (0.5); compare to normal detents which use a snap point *past* the next value (i.e. > 1)
        "On/off\nStrong detent",
        0,
        {},
        0,
        157,
    },
    {
        127,
        0,
        5,
        0,
        -1,
        1 * PI / 2,
        2,
        1,
        1.1,
        "Fine values\nWith detents",
        0,
        {},
        0,
        25,
    },
    {
        0,
        0,
        4,
        0,
        0,
        60 * PI / 180,
        0.01,
        0.6,
        1.1,
        "Return-to-center",
        0,
        {},
        0,
        45,
    },
    {
        0,
        0,
        6,
        0,
        -1,
        8.181818182 * PI / 180,
        1,
        1,
        1.1,
        "Coarse values\nStrong detents",
        0,
        {},
        0,
        200,
    },
};
