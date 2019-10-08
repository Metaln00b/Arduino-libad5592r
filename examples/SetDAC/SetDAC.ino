#include <libad5592r.h>

ad5592r_t ad5592r;

void setup() {
    Serial.begin (115200);
    ad5592r_init(&ad5592r);
    ad5592r_reset(&ad5592r);
    ad5592r_setup_source_ref_set(&ad5592r, AD5592R_VREF_SOURCE_INTERNAL);
    ad5592r_setup_double_vref_set(&ad5592r, true);
    ad5592r_setup_supply_voltage_mV(&ad5592r, 3300);

    ad5592r_setup_dac_all(&ad5592r, AD5592R_IO(0));
    ad5592r_setup_ldac(&ad5592r, AD5592R_LDAC_MODE_IMMEDIATELY);
}

void loop() {
    if (Serial.available() > 0) {
        double voltage_mV_set = Serial.parseInt();
    
        Serial.print("You send: ");
        Serial.print(voltage_mV_set);
        Serial.println();
        ad5592r_dac_voltage_set_mV(&ad5592r, AD5592R_IO(0), voltage_mV_set);
    }
}