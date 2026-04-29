#include <homekit/homekit.h>
#include <homekit/characteristics.h>

void identify(homekit_value_t value);

homekit_characteristic_t cha_name =
    HOMEKIT_CHARACTERISTIC_(NAME, "空调");

homekit_characteristic_t cha_active =
    HOMEKIT_CHARACTERISTIC_(ACTIVE, 0);

homekit_characteristic_t cha_current_temp =
    HOMEKIT_CHARACTERISTIC_(
        CURRENT_TEMPERATURE,
        26,
        .min_value = (float[]) {0},
        .max_value = (float[]) {50},
        .min_step = (float[]) {0.1}
    );

homekit_characteristic_t cha_current_state =
    HOMEKIT_CHARACTERISTIC_(CURRENT_HEATER_COOLER_STATE, 0);

homekit_characteristic_t cha_target_state =
    HOMEKIT_CHARACTERISTIC_(TARGET_HEATER_COOLER_STATE, 2);

homekit_characteristic_t cha_cooling_threshold =
    HOMEKIT_CHARACTERISTIC_(
        COOLING_THRESHOLD_TEMPERATURE,
        26,
        .min_value = (float[]) {16},
        .max_value = (float[]) {30},
        .min_step = (float[]) {1}
    );

homekit_characteristic_t cha_heating_threshold =
    HOMEKIT_CHARACTERISTIC_(
        HEATING_THRESHOLD_TEMPERATURE,
        25,
        .min_value = (float[]) {16},
        .max_value = (float[]) {30},
        .min_step = (float[]) {1}
    );

homekit_characteristic_t cha_temp_units =
    HOMEKIT_CHARACTERISTIC_(TEMPERATURE_DISPLAY_UNITS, 0);

homekit_characteristic_t cha_rotation_speed =
    HOMEKIT_CHARACTERISTIC_(
        ROTATION_SPEED,
        0,
        .min_value = (float[]) {0},
        .max_value = (float[]) {100},
        .min_step = (float[]) {1}
    );

homekit_characteristic_t cha_swing_mode =
    HOMEKIT_CHARACTERISTIC_(SWING_MODE, 0);

homekit_characteristic_t cha_display_light_name =
    HOMEKIT_CHARACTERISTIC_(NAME, "面板指示灯");

homekit_characteristic_t cha_display_light_on =
    HOMEKIT_CHARACTERISTIC_(ON, false);

homekit_characteristic_t cha_firmware_revision =
    HOMEKIT_CHARACTERISTIC_(FIRMWARE_REVISION, "00000000");

homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(
        .id = 1,
        .category = homekit_accessory_category_air_conditioner,
        .services = (homekit_service_t*[]) {
            HOMEKIT_SERVICE(
                ACCESSORY_INFORMATION,
                .characteristics = (homekit_characteristic_t*[]) {
                    HOMEKIT_CHARACTERISTIC(NAME, "空调"),
                    HOMEKIT_CHARACTERISTIC(MANUFACTURER, "Han"),
                    HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "8266"),
                    HOMEKIT_CHARACTERISTIC(MODEL, "8266-IRAC"),
                    &cha_firmware_revision,
                    HOMEKIT_CHARACTERISTIC(IDENTIFY, identify),
                    NULL
                }
            ),
            HOMEKIT_SERVICE(
                HEATER_COOLER,
                .primary = true,
                .characteristics = (homekit_characteristic_t*[]) {
                    &cha_name,
                    &cha_active,
                    &cha_current_temp,
                    &cha_current_state,
                    &cha_target_state,
                    &cha_cooling_threshold,
                    &cha_heating_threshold,
                    &cha_temp_units,
                    &cha_rotation_speed,
                    &cha_swing_mode,
                    NULL
                }
            ),
            HOMEKIT_SERVICE(
                LIGHTBULB,
                .characteristics = (homekit_characteristic_t*[]) {
                    &cha_display_light_name,
                    &cha_display_light_on,
                    NULL
                }
            ),
            NULL
        }
    ),
    NULL
};

homekit_server_config_t config = {
    .accessories = accessories,
    .password = "147-25-836"
};