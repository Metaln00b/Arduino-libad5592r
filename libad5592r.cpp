#include <Arduino.h>
#include <SPI.h>

#include "libad5592r.h"
#include "libad5592r.private.h"

#define DEBUG       false


#define AD5592R_SPI_CLOCK (4000000)

ad5592r_t *
ad5592r_init(ad5592r_t *obj)
{
    pinMode(SS, OUTPUT);
    digitalWrite(SS, LOW);

    SPI.begin();
    SPI.beginTransaction(SPISettings(AD5592R_SPI_CLOCK, MSBFIRST, SPI_MODE1)) ;
  
    obj->supply_voltage_mV = 3300;         /* Voreingestellt Versorgungsspannung (Vdd) */
    obj->external_ref_voltage_mV = 3300;   /* Voreingestellte externe Referenzsspannung (Vref) */

    return obj;
}

void
ad5592r_reset(ad5592r_t *obj)
{
    uint8_t receive[2];
    ad5592r_comm(obj, AD5592R_CMD_SW_RESET, receive);
}


/*******************************************************************************
 * Hilfsfunktionen                                                             *
 ******************************************************************************/

uint16_t
ad5592r_analog_2_digital(ad5592r_t *obj, double voltage_mV, bool double_vref)
{
    uint16_t sample;
    double precalc;
    uint16_t ref_voltage_mV;


    if (ad5592r_setup_source_ref_get(obj) == AD5592R_VREF_SOURCE_INTERNAL)
    {
        ref_voltage_mV = AD5592R_INT_REF_mV;
    }
    else
    {
        ref_voltage_mV = obj->external_ref_voltage_mV;
    }

    if (double_vref == true)
    {
        ref_voltage_mV = 2*ref_voltage_mV;
    }

    precalc = (AD5592R_SAMPLE_CODE_MAX / 1.00) / (ref_voltage_mV / 1.00);

    sample = voltage_mV * precalc;

    return sample;
}

double
ad5592r_digital_2_analog(ad5592r_t *obj, uint16_t sample, bool double_vref)
{
    double voltage_mV;
    double precalc;
    double ref_voltage_mV;

    if (ad5592r_setup_source_ref_get(obj) == AD5592R_VREF_SOURCE_INTERNAL)
    {
        ref_voltage_mV = AD5592R_INT_REF_mV;
    }
    else
    {
        ref_voltage_mV = obj->external_ref_voltage_mV;
    }

    if (double_vref == true)
    {
        ref_voltage_mV = 2*ref_voltage_mV;
    }

    precalc = (AD5592R_SAMPLE_CODE_MAX / 1.00) / (ref_voltage_mV / 1.00);

    voltage_mV = sample / precalc;

    return voltage_mV;
}

uint16_t
ad5592r_percentage_2_digital(unsigned int percentage)
{
    uint16_t sample = percentage * (AD5592R_SAMPLE_CODE_MAX / 100);
    return sample;
}

double
ad5592r_percentage_2_analog(ad5592r_t *obj, unsigned int percentage)
{
    double voltage_mV;

    voltage_mV = percentage * (obj->supply_voltage_mV / 100);

    return voltage_mV;
}

unsigned int
ad5592r_analog_2_percentage(ad5592r_t *obj, double voltage_mV)
{
    unsigned int percentage;

    percentage = voltage_mV / (obj->supply_voltage_mV / 100);

    return percentage;
}

void
ad5592r_split_word(uint8_t eight_bits[], ad5592r_word sixteen_bits, size_t arr_size)
{
    memset(eight_bits, 0x00, arr_size);
    eight_bits[0] = (sixteen_bits & 0xFF00) >> 8;
    eight_bits[1] = sixteen_bits & 0xFF;
}

uint8_t
ad5592r_macro_2_pin(uint8_t macro)
{
    uint8_t pin;

    switch(macro)
    {
        case   1: pin = 0; break;
        case   2: pin = 1; break;
        case   4: pin = 2; break;
        case   8: pin = 3; break;
        case  16: pin = 4; break;
        case  32: pin = 5; break;
        case  64: pin = 6; break;
        case 128: pin = 7; break;
        default:  pin = 0; break;
    }
    return pin;
}

/*******************************************************************************
 * Kommunikation                                                               *
 ******************************************************************************/

bool
ad5592r_comm(ad5592r_t *obj __attribute__((unused)), ad5592r_word sixteen_bits, uint8_t *rx_data)
{
    uint8_t send[2];
    ad5592r_split_word(send, sixteen_bits, sizeof(send));

    digitalWrite(SS, LOW);

    for (size_t i = 0; i < sizeof(send); i++)
    {
        rx_data[i] = SPI.transfer(send[i]);
    }
#if DEBUG == true
    Serial.println("SEND:");
    Serial.println(sixteen_bits, HEX);
    Serial.println("REC:");
    Serial.println(rx_data[0], HEX);
    Serial.println(rx_data[1], HEX);
#endif
    digitalWrite(SS, HIGH);

    return true;
}

/*******************************************************************************
 * Allgemeine Konfiguration                                                    *
 ******************************************************************************/

uint8_t
ad5592r_register_readback(ad5592r_t *obj, ad5592r_reg_readback_t reg)
{
    uint8_t receive[2];
    ad5592r_comm(obj, AD5592R_CMD_CNTRL_REG_READBACK | 0x41 | (reg << 2), receive);
    ad5592r_comm(obj, AD5592R_CMD_NOP, receive);

    ad5592r_setup_ldac(obj, obj->ldac_mode);   /* Alten LDAC-Modus wiederherstellen */

    return receive[1];
}

bool
ad5592r_setup_supply_voltage_mV(ad5592r_t *obj, double voltage_mV)
{
    if ( (voltage_mV >= AD5592R_MIN_VDD_mV) && (voltage_mV <= AD5592R_MAX_VDD_mV) )
    {
        obj->supply_voltage_mV = voltage_mV;
        return true;
    }
    else
    {
        return false;
    }
}

void
ad5592r_setup_source_ref_set(ad5592r_t *obj, ad5592r_vref_source_t source)
{
    uint8_t receive[2];

    if (source == AD5592R_VREF_SOURCE_INTERNAL) {

        obj->vref_source = AD5592R_VREF_SOURCE_INTERNAL;

        ad5592r_comm(obj, AD5592R_CMD_POWER_DWN_REF_CNTRL | (0x2 << 8), receive);
    }
    if (source == AD5592R_VREF_SOURCE_EXTERNAL) {

        obj->vref_source = AD5592R_VREF_SOURCE_EXTERNAL;

        ad5592r_comm(obj, AD5592R_CMD_POWER_DWN_REF_CNTRL, receive);
    }
}

bool
ad5592r_setup_external_ref_voltage_mV(ad5592r_t *obj, double voltage_mV)
{
    double vdd_voltage_mV;

    vdd_voltage_mV = obj->supply_voltage_mV;

    if ( (voltage_mV >= AD5592R_MIN_EXT_REF_mV) && (voltage_mV <= vdd_voltage_mV) )
    {
        obj->external_ref_voltage_mV = voltage_mV;
        return true;
    }
    else
    {
        return false;
    }
}

ad5592r_vref_source_t
ad5592r_setup_source_ref_get(ad5592r_t *obj)
{
    ad5592r_vref_source_t result;

    result = obj->vref_source;

    return result;
}

void
ad5592r_setup_double_vref_set(ad5592r_t *obj, bool double_vref)
{
    uint8_t receive[2];

    if (double_vref == true)
    {
        obj->double_vref_adc = true;
        obj->double_vref_dac = true;

        ad5592r_comm(obj, AD5592R_CMD_GP_CNTRL | AD5592R_ADC_TT_VREF | AD5592R_DAC_TT_VREF, receive);
    }
    if (double_vref == false)
    {
        obj->double_vref_adc = false;
        obj->double_vref_dac = false;

        ad5592r_comm(obj, AD5592R_CMD_GP_CNTRL, receive);
    }
}

bool
ad5592r_setup_double_vref_adc_get(ad5592r_t *obj)
{
    bool result;

    result = obj->double_vref_adc;

    return result;
}

bool
ad5592r_setup_double_vref_dac_get(ad5592r_t *obj)
{
    bool result;

    result = obj->double_vref_dac;

    return result;
}


/*******************************************************************************
 * Pin Konfiguration                                                           *
 ******************************************************************************/

void
ad5592r_setup_ldac(ad5592r_t *obj, ad5592r_ldac_mode_t ldac_mode)
{
    uint8_t receive[2];

    obj->ldac_mode = ldac_mode;
    ad5592r_comm(obj, AD5592R_CMD_CNTRL_REG_READBACK | ldac_mode, receive);
}

void
ad5592r_setup_dac_all(ad5592r_t *obj, uint8_t pins)
{
    uint8_t receive[2];

    for (int i = AD5592R_CHANNEL_MIN; i <= AD5592R_CHANNEL_MAX; i++) {
        if ( (pins & AD5592R_IO(i)) == AD5592R_IO(i))
        {
            obj->io_setting[i].type = AD5592R_IO_TYPE_DAC;
        }
    }
    ad5592r_comm(obj, AD5592R_CMD_DAC_PIN_SELECT | pins, receive);
}

void
ad5592r_setup_dac(ad5592r_t *obj, uint8_t pin)
{
    uint8_t receive[2];
    uint8_t pins = 0x0;

    pin = ad5592r_macro_2_pin(pin);

    obj->io_setting[pin].type = AD5592R_IO_TYPE_DAC;

    for (int i = AD5592R_CHANNEL_MIN; i <= AD5592R_CHANNEL_MAX; i++)
    {
        if (obj->io_setting[i].type == AD5592R_IO_TYPE_DAC)
        {
            pins |= (1 << i);
        }
    }
    ad5592r_comm(obj, AD5592R_CMD_DAC_PIN_SELECT | pins, receive);
}

void
ad5592r_setup_adc_all(ad5592r_t *obj, uint8_t pins)
{
    uint8_t receive[2];

    for (int i = AD5592R_CHANNEL_MIN; i <= AD5592R_CHANNEL_MAX; i++) {
        if ( (pins & AD5592R_IO(i)) == AD5592R_IO(i))
        {
            obj->io_setting[i].type = AD5592R_IO_TYPE_ADC;
        }
    }
    ad5592r_comm(obj, AD5592R_CMD_ADC_PIN_SELECT | pins, receive);
}

void
ad5592r_setup_adc(ad5592r_t *obj, uint8_t pin)
{
    uint8_t receive[2];
    uint8_t pins = 0x0;

    pin = ad5592r_macro_2_pin(pin);

    obj->io_setting[pin].type = AD5592R_IO_TYPE_ADC;

    for (int i = AD5592R_CHANNEL_MIN; i <= AD5592R_CHANNEL_MAX; i++)
    {
        if (obj->io_setting[i].type == AD5592R_IO_TYPE_ADC)
        {
            pins |= (1 << i);
        }
    }
    ad5592r_comm(obj, AD5592R_CMD_ADC_PIN_SELECT | pins, receive);
}

void
ad5592r_gpio_setup_type_all(ad5592r_t *obj, uint8_t pins, ad5592r_gpio_type_t type)
{
    uint8_t receive[2];
    uint8_t types = 0x0;

    for (int i = AD5592R_CHANNEL_MIN; i <= AD5592R_CHANNEL_MAX; i++) {
        if ( (pins & AD5592R_IO(i)) == AD5592R_IO(i))
        {
            obj->io_setting[i].gpio.type = type;
            if (obj->io_setting[i].gpio.type == AD5592R_GPIO_TYPE_OPEN_DRAIN)
            {
                types |= (1 << i);
            }
        }
    }
    ad5592r_comm(obj, AD5592R_CMD_GPIO_DRAIN_CONFIG | types, receive);
}

void
ad5592r_gpio_setup_type(ad5592r_t *obj, uint8_t pin, ad5592r_gpio_type_t type)
{
    uint8_t receive[2];
    uint8_t types = 0x0;

    pin = ad5592r_macro_2_pin(pin);

    obj->io_setting[pin].gpio.type = type;

    for (int i = AD5592R_CHANNEL_MIN; i <= AD5592R_CHANNEL_MAX; i++)
    {
        if (obj->io_setting[i].gpio.type == AD5592R_GPIO_TYPE_OPEN_DRAIN)
        {
            types |= (1 << i);
        }
    }
    ad5592r_comm(obj, AD5592R_CMD_GPIO_DRAIN_CONFIG | types, receive);
}

void
ad5592r_gpio_pulldown_all_set(ad5592r_t *obj, uint8_t pins, bool state)
{
    uint8_t receive[2];

    for (int i = AD5592R_CHANNEL_MIN; i <= AD5592R_CHANNEL_MAX; i++) {
        if ( (pins & AD5592R_IO(i)) == AD5592R_IO(i))
        {
            obj->io_setting[i].gpio.pull_down = state;
            if (obj->io_setting[i].gpio.pull_down == true)
            {
                pins |= (1 << i);
            }
        }
    }
    ad5592r_comm(obj, AD5592R_CMD_PULL_DOWN_SET | pins, receive);
}

void
ad5592r_gpio_pulldown_set(ad5592r_t *obj, uint8_t pin, bool state)
{
    uint8_t receive[2];
    uint8_t pins = 0x0;

    pin = ad5592r_macro_2_pin(pin);

    obj->io_setting[pin].gpio.pull_down = state;

    for (int i = AD5592R_CHANNEL_MIN; i <= AD5592R_CHANNEL_MAX; i++)
    {
        if (obj->io_setting[i].gpio.pull_down == true)
        {
            pins |= (1 << i);
        }
    }

    ad5592r_comm(obj, AD5592R_CMD_PULL_DOWN_SET | pins, receive);
}

void
ad5592r_gpio_direction_all_set(ad5592r_t *obj, uint8_t pins, ad5592r_gpio_direction_t direction)
{
    uint8_t receive[2];
    uint8_t directions = 0x0;

    for (int i = AD5592R_CHANNEL_MIN; i <= AD5592R_CHANNEL_MAX; i++) {
        if ( (pins & AD5592R_IO(i)) == AD5592R_IO(i))
        {
            obj->io_setting[i].type = AD5592R_IO_TYPE_GPIO;
            obj->io_setting[i].gpio.direction = direction;
            directions |= (1 << i);
        }
    }

    if (direction == AD5592R_GPIO_DIRECTION_IN)
    {
        ad5592r_comm(obj, AD5592R_CMD_GPIO_READ_CONFIG | directions, receive);
    }
    if (direction == AD5592R_GPIO_DIRECTION_OUT)
    {
        ad5592r_comm(obj, AD5592R_CMD_GPIO_WRITE_CONFIG | directions, receive);
    }
}

void
ad5592r_gpio_direction_set(ad5592r_t *obj, uint8_t pin, ad5592r_gpio_direction_t direction)
{
    uint8_t receive[2];
    uint8_t directions = 0x0;

    pin = ad5592r_macro_2_pin(pin);

    obj->io_setting[pin].gpio.direction = direction;
    obj->io_setting[pin].type = AD5592R_IO_TYPE_GPIO;

    for (int i = AD5592R_CHANNEL_MIN; i <= AD5592R_CHANNEL_MAX; i++)
    {
        if (obj->io_setting[i].gpio.direction == direction)
        {
            directions |= (1 << i);
        }
    }

    if (direction == AD5592R_GPIO_DIRECTION_IN)
    {
        ad5592r_comm(obj, AD5592R_CMD_GPIO_READ_CONFIG | directions, receive);
    }
    if (direction == AD5592R_GPIO_DIRECTION_OUT)
    {
        ad5592r_comm(obj, AD5592R_CMD_GPIO_WRITE_CONFIG | directions, receive);
    }
}

void
ad5592r_gpio_state_all_set(ad5592r_t *obj, uint8_t pins, ad5592r_gpio_state_t state)
{
    uint8_t receive[2];
    uint8_t pp_states = 0x0;
    uint8_t z_states = 0x0;

    for (int i = AD5592R_CHANNEL_MIN; i <= AD5592R_CHANNEL_MAX; i++) {
        if ( (pins & AD5592R_IO(i)) == AD5592R_IO(i))
        {
            obj->io_setting[i].gpio.state = state;
            if (obj->io_setting[i].gpio.state == AD5592R_GPIO_HIGH)
            {
                pp_states |= (1 << i);
            }
            if (obj->io_setting[i].gpio.state == AD5592R_GPIO_Z)
            {
                z_states |= (1 << i);
            }
        }
    }

    ad5592r_comm(obj, AD5592R_CMD_GPIO_WRITE_DATA | pp_states, receive);
    ad5592r_comm(obj, AD5592R_CMD_THREE_STATE_CONFIG | z_states, receive);

}

void
ad5592r_gpio_state_set(ad5592r_t *obj, uint8_t pin, ad5592r_gpio_state_t state)
{
    uint8_t receive[2];
    uint8_t pp_states = 0x0;
    uint8_t z_states = 0x0;

    pin = ad5592r_macro_2_pin(pin);

    obj->io_setting[pin].gpio.state = state;

    for (int i = AD5592R_CHANNEL_MIN; i <= AD5592R_CHANNEL_MAX; i++)
    {
        if (obj->io_setting[i].gpio.state == AD5592R_GPIO_HIGH)
        {
            pp_states |= (1 << i);
        }
        if (obj->io_setting[i].gpio.state == AD5592R_GPIO_Z)
        {
            z_states |= (1 << i);
        }
    }

    ad5592r_comm(obj, AD5592R_CMD_GPIO_WRITE_DATA | pp_states, receive);
    ad5592r_comm(obj, AD5592R_CMD_THREE_STATE_CONFIG | z_states, receive);
}

uint8_t
ad5592r_gpio_input_state_get(ad5592r_t *obj, uint8_t pins)
{
    uint8_t receive[2];

    ad5592r_comm(obj, AD5592R_CMD_GPIO_READ_INPUT | pins, receive);
    ad5592r_comm(obj, AD5592R_CMD_NOP, receive);

    return receive[1];
}

uint8_t
ad5592r_gpio_output_state_get(ad5592r_t *obj, uint8_t pin)
{
    uint8_t result;

    pin = ad5592r_macro_2_pin(pin);

    result = obj->io_setting[pin].gpio.state;

    return result;
}

uint16_t
ad5592r_adc_sample_get(ad5592r_t *obj, uint8_t pin)
{
    uint8_t receive[2];

    pin = ad5592r_macro_2_pin(pin);

    ad5592r_comm(obj, AD5592R_CMD_ADC_READ | (0x1 << pin), receive);
    ad5592r_comm(obj, AD5592R_CMD_NOP, receive);
    ad5592r_comm(obj, AD5592R_CMD_NOP, receive);

    /* Verwirft das erste Nibble */
    uint16_t result = ((receive[0] << 8) & 0x0F00) | (receive[1] & 0xFF);

    return result;
}

double
ad5592r_adc_voltage_get_mV(ad5592r_t *obj, uint8_t pin)
{
    uint8_t receive[2];

    pin = ad5592r_macro_2_pin(pin);

    ad5592r_comm(obj, AD5592R_CMD_ADC_READ | (0x1 << pin), receive);
    ad5592r_comm(obj, AD5592R_CMD_NOP, receive);
    ad5592r_comm(obj, AD5592R_CMD_NOP, receive);

    /* Verwirft das erste Nibble */
    uint16_t result = ((receive[0] << 8) & 0x0F00) | (receive[1] & 0xFF);

    return ad5592r_digital_2_analog(obj, result, ad5592r_setup_double_vref_adc_get(obj));
}

unsigned int
ad5592r_adc_percentage_get(ad5592r_t *obj, uint8_t pin)
{
    return ad5592r_analog_2_percentage(obj, ad5592r_adc_voltage_get_mV(obj, pin) );
}

bool
ad5592r_dac_sample_set(ad5592r_t *obj, uint8_t pin, uint16_t sample)
{
    uint8_t receive[2];

    pin = ad5592r_macro_2_pin(pin);

    if (sample <= AD5592R_SAMPLE_CODE_MAX)
    {
        ad5592r_comm(obj,
            AD5592R_DAC_WRITE_MASK |                            /* DAC Schreibbefehl */
            ( (pin << 12) & AD5592R_DAC_ADDRESS_MASK ) |        /* DAC-Pin(Adresse) & DAC-Addressmaske */
            sample,                                             /* Digitalwert */
        receive);

        obj->io_setting[pin].dac.sample = sample;

        return true;
    }
    else
    {
        return false;
    }
}

bool
ad5592r_dac_voltage_set_mV(ad5592r_t *obj, uint8_t pin, double voltage_mV)
{
    uint8_t receive[2];
    uint16_t result;
    double supply_voltage_mV;

    pin = ad5592r_macro_2_pin(pin);

    supply_voltage_mV = obj->supply_voltage_mV;

    if (voltage_mV <= supply_voltage_mV)
    {
        result = ad5592r_analog_2_digital(obj, voltage_mV, ad5592r_setup_double_vref_dac_get(obj));
        ad5592r_comm(obj,
            AD5592R_DAC_WRITE_MASK |                            /* DAC Schreibbefehl */
            ( (pin << 12) & AD5592R_DAC_ADDRESS_MASK ) |        /* DAC-Pin(Adresse) & DAC-Addressmaske */
            result,                                             /* Digitalwert */
        receive);

        obj->io_setting[pin].dac.sample = result;

        return true;
    }
    else
    {
        return false;
    }
}

bool
ad5592r_dac_percentage_set(ad5592r_t *obj, uint8_t pin, unsigned int percentage)
{
    if (percentage <= 100)
    {
        double voltage_mV = ad5592r_percentage_2_analog(obj, percentage);
        ad5592r_dac_voltage_set_mV(obj, pin, voltage_mV);

        return true;
    }
    else
    {
        return false;
    }
}

uint16_t
ad5592r_dac_sample_get(ad5592r_t *obj, uint8_t pin)
{
    uint16_t result;

    pin = ad5592r_macro_2_pin(pin);

    result = obj->io_setting[pin].dac.sample;

    return result;
}


/*******************************************************************************
 * Sonstige                                                                    *
 ******************************************************************************/

double
ad5592r_temperature_get_degC(ad5592r_t *obj)
{
    uint8_t receive[2];
    double result;
    double vref_mV = 0;
    ad5592r_vref_source_t vref_source;

    ad5592r_comm(obj, AD5592R_CMD_ADC_READ | (0x1 << 8), receive);
    delayMicroseconds(25);
    ad5592r_comm(obj, AD5592R_CMD_NOP, receive);
    ad5592r_comm(obj, AD5592R_CMD_NOP, receive);

    /* Verwirft das erste Nibble */
    uint16_t temp_raw = ((receive[0] << 8) & 0x0F00) | (receive[1] & 0xFF);

    vref_source = obj->vref_source;

    if (vref_source == AD5592R_VREF_SOURCE_EXTERNAL)
    {
        vref_mV = obj->external_ref_voltage_mV;
    }
    if (vref_source == AD5592R_VREF_SOURCE_INTERNAL)
    {
        vref_mV = AD5592R_INT_REF_mV;
    }

    if (ad5592r_setup_double_vref_adc_get(obj) == true)
    {
        result = AD5592R_TEMPERATURE_FORMULA_GAIN2(temp_raw, vref_mV);
    }
    else
    {
        result = AD5592R_TEMPERATURE_FORMULA_GAIN1(temp_raw, vref_mV);
    }

    return result;
}

uint16_t
ad5592r_temperature_sample_get(ad5592r_t *obj)
{
    uint8_t receive[2];

    ad5592r_comm(obj, AD5592R_CMD_ADC_READ | (0x1 << 8), receive);
    ad5592r_comm(obj, AD5592R_CMD_NOP, receive);
    ad5592r_comm(obj, AD5592R_CMD_NOP, receive);

    /* Verwirft das erste Nibble */
    uint16_t result = ((receive[0] << 8) & 0x0F00) | (receive[1] & 0xFF);

    return result;
}

double
ad5592r_sample_to_temperature_degC(ad5592r_t *obj, uint16_t temperature_sample)
{
    double result;
    double vref_mV = 0;
    ad5592r_vref_source_t vref_source;

    vref_source = obj->vref_source;

    if (vref_source == AD5592R_VREF_SOURCE_EXTERNAL)
    {
        vref_mV = obj->external_ref_voltage_mV;
    }
    if (vref_source == AD5592R_VREF_SOURCE_INTERNAL)
    {
        vref_mV = AD5592R_INT_REF_mV;
    }

    if (ad5592r_setup_double_vref_adc_get(obj) == true)
    {
        result = AD5592R_TEMPERATURE_FORMULA_GAIN2(temperature_sample, vref_mV);
    }
    else
    {
        result = AD5592R_TEMPERATURE_FORMULA_GAIN1(temperature_sample, vref_mV);
    }

    return result;
}
