#include <Arduino.h>
#include <EEPROM.h>
#include <TFT_eSPI.h>
#include <HX711.h>
#include "util.h"
#include "BLDCMotor.h"
#include "drivers/BLDCDriver6PWM.h"
#include "mt6701_sensor.h"
// #include "configuration.h"
#include <pb.h>

#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>

#include "main.h"

// SSID & Passwort für das lokale WiFi
#define WIFI_SSID "FRITZ!Box6591CableMH"
#define WIFI_PASSWORD "76111069267883455750"
#define FOC_PID_P 4
#define FOC_PID_I 0
#define FOC_PID_D 0.04
#define FOC_PID_OUTPUT_RAMP 10000
#define FOC_PID_LIMIT 10
#define FOC_VOLTAGE_LIMIT 5

#define X_CENTER_CIRCLE 120
#define Y_CENTER_CIRCLE 160
#define RADIUS_CIRCLE 100

#define DETENT_LEFT (25 / 18) * PI
#define DETENT_RIGHT (29 / 18) * PI
#define DETENT_MIDDLE (3 / 2) * PI

// Function Prototypes
void connectToWiFi();
// void MotorRun(bool pressstatus);
void CalibrationKlick();
void CalibrateSensorZero();
void HapticClick();
//  void DrawPointer(float angle);
//  float LoopSensorValue(float meas_val);
//  void MotorCalibrate();
void CommandTypeFunc(Command command);

// Task function prototypes
void task1Function(void *pvParameters);
void task2Function(void *pvParameters);

// Task handles
TaskHandle_t Task1;
TaskHandle_t Task2;

// Definitions
TFT_eSPI tft_ = TFT_eSPI();
TFT_eSprite spr_ = TFT_eSprite(&tft_);
HX711 klick;
BLDCMotor motor = BLDCMotor(1);
BLDCDriver6PWM driver = BLDCDriver6PWM(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL);
MT6701Sensor encoder = MT6701Sensor();
// Configuration& configuration_;

// Variables
static const uint8_t LEDC_CHANNEL_LCD_BACKLIGHT = 0;
bool run = 1;
int idle_klick = 0;
int delta_klick = 0;
float click = 0;
char buf_[72];
static bool pressed = pdFALSE;
static bool pressed_before = pdFALSE;
uint8_t press_count_ = 1;
// Variables - Calibrate Sensor to 0
float old_val = 0;
float new_val = 0;
float dif_val = 0;
float cal_val = 0;
// Variables - EEProm
int eepromAddress = 0;    // Define the EEPROM address where the variable will be stored
int CalibrationClick = 0; // Define the variable to be set and saved
// Variables - Kreis
float x_circle = 0;
float y_circle = 0;
// Variables - test2
float idle_check_velocity_ewma = 0;
uint32_t last_idle_start = 0;
uint32_t last_publish = 0;
static const float DEAD_ZONE_DETENT_PERCENT = 0.2;
static const float DEAD_ZONE_RAD = 1 * _PI / 180;
static const float IDLE_VELOCITY_EWMA_ALPHA = 0.001;
static const float IDLE_VELOCITY_RAD_PER_SEC = 0.05;
static const uint32_t IDLE_CORRECTION_DELAY_MILLIS = 500;
static const float IDLE_CORRECTION_MAX_ANGLE_RAD = 5 * PI / 180;
static const float IDLE_CORRECTION_RATE_ALPHA = 0.0005;

float current_detent_center;
float prevValue = 0.0;
float sensorValue = 0.0;

PB_SmartKnobConfig config = {
    .position = 0,
    .sub_position_unit = 0,
    .position_nonce = 0,
    .min_position = 0,
    .max_position = 1,
    .position_width_radians = 60 * _PI / 180,
    .detent_strength_unit = 0,
};
int32_t current_position = 0;
float latest_sub_position_unit = 0;

int i = 0;
bool switching = false;

Command command;

void setup()
{
    Serial.begin(921600);
    EEPROM.begin(512);
    delta_klick = EEPROM.readInt(eepromAddress);
    eepromAddress += sizeof(delta_klick);
    idle_klick = EEPROM.readInt(eepromAddress);
    eepromAddress = 0;
    EEPROM.end();
    // connectToWiFi();
    Serial.println("Connected");
    tft_.begin();
    tft_.invertDisplay(1);
    tft_.setRotation(SK_DISPLAY_ROTATION);
    tft_.fillScreen(TFT_LIGHTGREY);
    klick.begin(38, 2);

    driver.voltage_power_supply = 5;
    driver.init();
    encoder.init();

    motor.linkDriver(&driver);
    motor.controller = MotionControlType::torque;
    motor.voltage_limit = FOC_VOLTAGE_LIMIT;
    motor.velocity_limit = 10000;
    motor.linkSensor(&encoder);
    motor.PID_velocity.P = FOC_PID_P;
    motor.PID_velocity.I = FOC_PID_I;
    motor.PID_velocity.D = FOC_PID_D;
    motor.PID_velocity.output_ramp = FOC_PID_OUTPUT_RAMP;
    motor.PID_velocity.limit = FOC_PID_LIMIT;
    motor.init();
    encoder.update();
    // motor.controller = MotionControlType::angle_openloop;
    motor.pole_pairs = 7;
    // motor.initFOC(5.515216, Direction::CW);
    // motor.monitor_downsample = 0;
    current_detent_center = motor.shaft_angle;

    ledcSetup(LEDC_CHANNEL_LCD_BACKLIGHT, 5000, 16);
    ledcAttachPin(PIN_LCD_BACKLIGHT, LEDC_CHANNEL_LCD_BACKLIGHT);
    ledcWrite(LEDC_CHANNEL_LCD_BACKLIGHT, UINT16_MAX);
    spr_.setColorDepth(8);
    if (spr_.createSprite(TFT_WIDTH, TFT_HEIGHT) == nullptr)
    {
        Serial.println("ERROR: sprite allocation failed!");
        tft_.fillScreen(TFT_RED);
    }
    else
    {
        Serial.println("Sprite created!");
        tft_.fillScreen(TFT_PURPLE);
    }
    // spr_.setTextColor(0xFFFF, TFT_BLACK);

    // const int RADIUS = TFT_WIDTH / 2;
    //  const uint16_t FILL_COLOR = spr_.color565(90, 18, 151);
    //  const uint16_t DOT_COLOR = spr_.color565(80, 100, 200);
    // spr_.fillCircle(TFT_WIDTH / 2, TFT_HEIGHT / 2, RADIUS / 2, TFT_RED);
    // spr_.drawLine(TFT_WIDTH / 2, TFT_HEIGHT / 2, TFT_WIDTH / 2, 0, TFT_RED);
    Serial.println("drawCircle");
    spr_.pushSprite(0, 0);
    ledcWrite(LEDC_CHANNEL_LCD_BACKLIGHT, UINT16_MAX);
    // MotorCalibrate();
    CalibrateSensorZero();
    // current_detent_center = motor.shaftAngle();
    motor.initFOC(5.50, Direction::CW);
    // Create Task 1
    xTaskCreate(
        task1Function, // Function to execute Task 1
        "Task1",       // Task name
        4096,          // Stack size (words)
        NULL,          // Parameter to pass to the task
        1,             // Task priority (0 to configMAX_PRIORITIES-1)
        &Task1         // Task handle
    );

    // Create Task 2
    /*    xTaskCreate(
            task2Function, // Function to execute Task 2
            "Task2",       // Task name
            1000,          // Stack size (words)
            NULL,          // Parameter to pass to the task
            1,             // Task priority (0 to configMAX_PRIORITIES-1)
            &Task2         // Task handle
        );*/
    //CalibrationKlick();
    Serial.println("RUN BITCH;");
}
// Task 1 function
void task1Function(void *pvParameters)
{
    command.command_type = CommandType::CONFIG;
    command.data.config = configs[0];
    for (;;)
    {
        CommandTypeFunc(command);
    }
}

void loop()
{
    // Command command;
    HapticClick();
    // CommandTypeFunc(command);
    //  click = lerp(klick.read(), idle_klick, idle_klick + delta_klick, 0, 1);
    //  Serial.println(klick.read());
}

void HapticClick()
{
    click = lerp(klick.read(), idle_klick, idle_klick + delta_klick, 0, 1);
    //  Ignore readings that are way out of expected bounds
    if (-1 < click && click < 2)
    {
        static uint8_t press_readings;
        if (!pressed && click > 1)
        {
            // Serial.println(press_readings);
            press_readings++;
            // Serial.println(press_readings);
            if (press_readings > 2)
            {
                command.command_type = CommandType::HAPTIC;
                command.data.haptic.press = true;
                pressed = true;
                press_count_++;
            }
        }
        else if (pressed && click < 0.5)
        {
            press_readings++;
            if (press_readings > 2)
            {
                command.command_type = CommandType::HAPTIC;
                command.data.haptic.press = false;
                pressed = false;
            }
        }
        else
        {
            command.command_type = CommandType::CONFIG;
            press_readings = 0;
        }
        if (pressed == 0 && pressed_before == 1)
        {
            if (switching)
            {
                command.data.config = configs[3];
                switching = !switching;
            }
            else if (!switching)
            {
                command.data.config = configs[0];
                switching = !switching;
            }
        }
        pressed_before = pressed;
    }
}

void connectToWiFi()
{
    int TryCount = 0;
    // log_i( "connect to wifi" );
    while (WiFi.status() != WL_CONNECTED)
    {
        TryCount++;
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        WiFi.hostname("smartViewKnob");
        vTaskDelay(4000);
        if (TryCount == 10)
        {
            ESP.restart();
        }
    }
    //  WiFi.onEvent( WiFiEvent );
} // void connectToWiFi()

void CalibrateSensorZero()
{
    old_val = encoder.getSensorAngle();
    new_val = encoder.getSensorAngle();
    // cal_val = (3*PI)/2;
    //  cal_val = (25 * PI) / 18; // Snap Point links 250°
    //  cal_val = (29*PI)/18;             // Snap Point rechts 290°
}

void CommandTypeFunc(Command command)
{
    motor.loopFOC();

    // Check queue for pending requests from other tasks
    // Command command;
    switch (command.command_type)
    {
    case CommandType::CALIBRATE:
        // calibrate();
        break;
    case CommandType::CONFIG:
    {
        // Check new config for validity
        PB_SmartKnobConfig &new_config = command.data.config;
        if (new_config.detent_strength_unit < 0)
        {
            Serial.println("Ignoring invalid config: detent_strength_unit cannot be negative");
            break;
        }
        if (new_config.endstop_strength_unit < 0)
        {
            Serial.println("Ignoring invalid config: endstop_strength_unit cannot be negative");
            break;
        }
        if (new_config.snap_point < 0.5)
        {
            Serial.println("Ignoring invalid config: snap_point must be >= 0.5 for stability");
            break;
        }
        if (new_config.detent_positions_count > COUNT_OF(new_config.detent_positions))
        {
            Serial.println("Ignoring invalid config: detent_positions_count is too large");
            break;
        }
        if (new_config.snap_point_bias < 0)
        {
            Serial.println("Ignoring invalid config: snap_point_bias cannot be negative or there is risk of instability");
            break;
        }

        // Change haptic input mode
        bool position_updated = false;
        if (new_config.position != config.position || new_config.sub_position_unit != config.sub_position_unit || new_config.position_nonce != config.position_nonce)
        {
            Serial.println("applying position change");
            current_position = new_config.position;
            position_updated = true;
        }

        if (new_config.min_position <= new_config.max_position)
        {
            // Only check bounds if min/max indicate bounds are active (min >= max)
            if (current_position < new_config.min_position)
            {
                current_position = new_config.min_position;
                Serial.println("adjusting position to min");
            }
            else if (current_position > new_config.max_position)
            {
                current_position = new_config.max_position;
                Serial.println("adjusting position to max");
            }
        }

        if (position_updated || new_config.position_width_radians != config.position_width_radians)
        {
            Serial.println("adjusting detent center");
            float new_sub_position = position_updated ? new_config.sub_position_unit : latest_sub_position_unit;
            Serial.println(new_sub_position);
#if SK_INVERT_ROTATION
            float shaft_angle = -motor.shaft_angle;
#else
            float shaft_angle = motor.shaft_angle;
#endif
            current_detent_center = shaft_angle + new_sub_position * new_config.position_width_radians;
        
        }
        config = new_config;

        // Update derivative factor of torque controller based on detent width.
        // If the D factor is large on coarse detents, the motor ends up making noise because the P&D factors amplify the noise from the sensor.
        // This is a piecewise linear function so that fine detents (small width) get a higher D factor and coarse detents get a small D factor.
        // Fine detents need a nonzero D factor to artificially create "clicks" each time a new value is reached (the P factor is small
        // for fine detents due to the smaller angular errors, and the existing P factor doesn't work well for very small angle changes (easy to
        // get runaway due to sensor noise & lag)).
        // TODO: consider eliminating this D factor entirely and just "play" a hardcoded haptic "click" (e.g. a quick burst of torque in each
        // direction) whenever the position changes when the detent width is too small for the P factor to work well.
        const float derivative_lower_strength = config.detent_strength_unit * 0.08;
        const float derivative_upper_strength = config.detent_strength_unit * 0.02;
        const float derivative_position_width_lower = radians(3);
        const float derivative_position_width_upper = radians(8);
        const float raw = derivative_lower_strength + (derivative_upper_strength - derivative_lower_strength) / (derivative_position_width_upper - derivative_position_width_lower) * (config.position_width_radians - derivative_position_width_lower);
        // When there are intermittent detents (set via detent_positions), disable derivative factor as this adds extra "clicks" when nearing
        // a detent.
        motor.PID_velocity.D = config.detent_positions_count > 0 ? 0 : CLAMP(raw, min(derivative_lower_strength, derivative_upper_strength), max(derivative_lower_strength, derivative_upper_strength));
        break;
    }
    case CommandType::HAPTIC:
    {
        // Play a hardcoded haptic "click"
        float strength = command.data.haptic.press ? 5 : 1.5;
        motor.move(strength);
        for (uint8_t i = 0; i < 3; i++)
        {
            motor.loopFOC();
            delay(1);
        }
        motor.move(-strength);
        for (uint8_t i = 0; i < 3; i++)
        {
            motor.loopFOC();
            delay(1);
        }
        motor.move(0);
        motor.loopFOC();
        break;
    }
    }

    // If we are not moving and we're close to the center (but not exactly there), slowly adjust the centerpoint to match the current position
    idle_check_velocity_ewma = motor.shaft_velocity * IDLE_VELOCITY_EWMA_ALPHA + idle_check_velocity_ewma * (1 - IDLE_VELOCITY_EWMA_ALPHA);
    if (fabsf(idle_check_velocity_ewma) > IDLE_VELOCITY_RAD_PER_SEC)
    {
        last_idle_start = 0;
    }
    else
    {
        if (last_idle_start == 0)
        {
            last_idle_start = millis();
        }
    }
    if (last_idle_start > 0 && millis() - last_idle_start > IDLE_CORRECTION_DELAY_MILLIS && fabsf(motor.shaft_angle - current_detent_center) < IDLE_CORRECTION_MAX_ANGLE_RAD)
    {
        current_detent_center = motor.shaft_angle * IDLE_CORRECTION_RATE_ALPHA + current_detent_center * (1 - IDLE_CORRECTION_RATE_ALPHA);
    }

    // Check where we are relative to the current nearest detent; update our position if we've moved far enough to snap to another detent
    float angle_to_detent_center = motor.shaft_angle - current_detent_center;
#if SK_INVERT_ROTATION
    angle_to_detent_center = -motor.shaft_angle - current_detent_center;
#endif

    float snap_point_radians = config.position_width_radians * config.snap_point;
    float bias_radians = config.position_width_radians * config.snap_point_bias;
    float snap_point_radians_decrease = snap_point_radians + (current_position <= 0 ? bias_radians : -bias_radians);
    float snap_point_radians_increase = -snap_point_radians + (current_position >= 0 ? -bias_radians : bias_radians);
    int32_t num_positions = config.max_position - config.min_position + 1;
    if (angle_to_detent_center > snap_point_radians_decrease && (num_positions <= 0 || current_position > config.min_position))
    {
        current_detent_center += config.position_width_radians;
        angle_to_detent_center -= config.position_width_radians;
        current_position--;
        //Serial.println(angle_to_detent_center);
    }
    else if (angle_to_detent_center < snap_point_radians_increase && (num_positions <= 0 || current_position < config.max_position))
    {
        current_detent_center -= config.position_width_radians;
        angle_to_detent_center += config.position_width_radians;
        current_position++;
        //Serial.println(angle_to_detent_center);
    }

    latest_sub_position_unit = -angle_to_detent_center / config.position_width_radians;

    float dead_zone_adjustment = CLAMP(
        angle_to_detent_center,
        fmaxf(-config.position_width_radians * DEAD_ZONE_DETENT_PERCENT, -DEAD_ZONE_RAD),
        fminf(config.position_width_radians * DEAD_ZONE_DETENT_PERCENT, DEAD_ZONE_RAD));

    bool out_of_bounds = num_positions > 0 && ((angle_to_detent_center > 0 && current_position == config.min_position) || (angle_to_detent_center < 0 && current_position == config.max_position));
    motor.PID_velocity.limit = 10; // out_of_bounds ? 10 : 3;
    motor.PID_velocity.P = out_of_bounds ? config.endstop_strength_unit * 4 : config.detent_strength_unit * 4;

    // Apply motor torque based on our angle to the nearest detent (detent strength, etc is handled by the PID_velocity parameters)
    if (fabsf(motor.shaft_velocity) > 60)
    {
        // Don't apply torque if velocity is too high (helps avoid positive feedback loop/runaway)
        motor.move(0);
    }
    else
    {
        float input = -angle_to_detent_center + dead_zone_adjustment;
        if (!out_of_bounds && config.detent_positions_count > 0)
        {
            bool in_detent = false;
            for (uint8_t i = 0; i < config.detent_positions_count; i++)
            {
                if (config.detent_positions[i] == current_position)
                {
                    in_detent = true;
                    break;
                }
            }
            if (!in_detent)
            {
                
                input = 0;
            }
        }
        float torque = motor.PID_velocity(input);
#if SK_INVERT_ROTATION
        torque = -torque;
#endif
        motor.move(torque);
        snprintf(buf_, sizeof(buf_), "torque: %.2f", torque);

    }

    delay(1);
}

void CalibrationKlick()
{
    Serial.println("Don't touch and klick 'c' ");
    while (Serial.available() <= 0)
    {
    }
    if (Serial.read() == 'c')
    {
        idle_klick = klick.read();
    }
    Serial.println("press with medium force and klick 'c' ");
    while (Serial.available() <= 0)
    {
    }
    if (Serial.read() == 'c')
    {
        delta_klick = klick.read() - idle_klick;
    }
    Serial.print("idle_klick: ");
    Serial.print(idle_klick);
    Serial.print("   delta_klick: ");
    Serial.println(CalibrationClick);
    //  }
    // Save the variable to EEPROM
    EEPROM.begin(512);
    EEPROM.writeInt(eepromAddress, delta_klick);
    eepromAddress += sizeof(delta_klick);
    EEPROM.writeInt(eepromAddress, idle_klick);
    EEPROM.commit();
    EEPROM.end();
    Serial.println("Saved Values");
}