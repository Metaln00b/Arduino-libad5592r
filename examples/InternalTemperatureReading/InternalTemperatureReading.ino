#include <libad5592r.h>

ad5592r_t ad5592r;

void setup() {
    Serial.begin (115200);
    ad5592r_init(&ad5592r);
    ad5592r_reset(&ad5592r);
    ad5592r_setup_source_ref_set(&ad5592r, AD5592R_VREF_SOURCE_INTERNAL);
    ad5592r_setup_double_vref_set(&ad5592r, true);
    ad5592r_setup_supply_voltage_mV(&ad5592r, 3300);
}

void loop() {
    size_t iteration_size = 50;
    double temp[iteration_size];
    double temp_sum = 0;
    double temp_avg;
    for (size_t i = 0; i < iteration_size; i++)
    {
        temp[i] = ad5592r_temperature_get_degC(&ad5592r);
        temp_sum += temp[i];
    }
    temp_avg = temp_sum/iteration_size;
    Serial.println(temp_avg, DEC);
    delay(100);
}