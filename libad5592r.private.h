/*******************************************************************************
* Konstanten                                                                   *
*******************************************************************************/

/* Bauteilspezifische Formeln für internen Temperatursensor */
#define AD5592R_TEMPERATURE_FORMULA_GAIN1(temp_raw, vref_mV) \
25 + ( (temp_raw - (0.5 / (vref_mV/1000)) * AD5592R_SAMPLE_CODE_MAX) / (2.654 * (2.5 / (vref_mV/1000))) )

#define AD5592R_TEMPERATURE_FORMULA_GAIN2(temp_raw, vref_mV) \
25 + ( (temp_raw - (0.5 / (2 * (vref_mV/1000))) * AD5592R_SAMPLE_CODE_MAX) / (1.327 * (2.5 / (vref_mV/1000))) )

/* Kontrollregister Definitionen */
#define AD5592R_CNTRL_ADDRESS_MASK      0x7800  /* Control register bit mask */

#define AD5592R_CMD_NOP                 0x0000  /* No operation */
#define AD5592R_CMD_DAC_READBACK        0x0800  /* Selects and enables DAC read back */
#define AD5592R_CMD_ADC_READ            0x1000  /* Selects ADCs for conversion */
#define AD5592R_CMD_GP_CNTRL            0x1800  /* General purpose control register */
#define AD5592R_CMD_ADC_PIN_SELECT      0x2000  /* Selects which pins are ADC inputs */
#define AD5592R_CMD_DAC_PIN_SELECT      0x2800  /* Selects which pins are DAC outputs */
#define AD5592R_CMD_PULL_DOWN_SET       0x3000  /* Selects which pins have 85kOhm pull-down resistor to GND */
#define AD5592R_CMD_CNTRL_REG_READBACK  0x3800  /* Read back control registers and/or set LDAC */
#define AD5592R_CMD_GPIO_WRITE_CONFIG   0x4000  /* Selects which pins are GPIO outputs */
#define AD5592R_CMD_GPIO_WRITE_DATA     0x4800  /* Writes data to the GPIO outputs */
#define AD5592R_CMD_GPIO_READ_CONFIG    0x5000  /* Selects which pins are GPIO inputs */
#define AD5592R_CMD_GPIO_READ_INPUT     0x5400  /* Read GPIO inputs */
#define AD5592R_CMD_POWER_DWN_REF_CNTRL 0x5800  /* Powers down DACs and enables/disables the reference */
#define AD5592R_CMD_GPIO_DRAIN_CONFIG   0x6000  /* Selects open-drain or push/pull for GPIO outputs */
#define AD5592R_CMD_THREE_STATE_CONFIG  0x6800  /* Selects which pins are three-state */
#define AD5592R_CMD_SW_RESET            0x7DAC  /* Software reset of the AD5592R */

#define AD5592R_PIN_SELECT_MASK         0x00FF  /* Pin select bit mask */

/* DAC Register Definitionen */
#define AD5592R_DAC_WRITE_MASK          0x8000  /* DAC write bit mask */
#define AD5592R_DAC_ADDRESS_MASK        0x7000  /* DAC pin address bit mask */
#define AD5592R_DAC_VALUE_MASK          0x0FFF  /* DAC output value bit mask */

/* Range Selection 2xVref */
#define AD5592R_ADC_TT_VREF             0x0010  /* Set ADC input range to 2 times Vref */
#define AD5592R_DAC_TT_VREF             0x0020  /* Set DAC output range to 2 times Vref */

/* Definitionen der Register-Readback-Adressen */
typedef enum {
    AD5592_REG_READBACK_NOP,
    AD5592_REG_READBACK_DAC_READBACK,
    AD5592_REG_READBACK_ADC_SEQ,
    AD5592_REG_READBACK_GPIO_CONF,
    AD5592_REG_READBACK_ADC_PIN_CONF,
    AD5592_REG_READBACK_DAC_PIN_CONF,
    AD5592_REG_READBACK_PULL_DOWN_CONF,
    AD5592_REG_READBACK_LDAC_CONF,
    AD5592_REG_READBACK_GPIO_WRITE_CONF,
    AD5592_REG_READBACK_GPIO_WRITE_DATA,
    AD5592_REG_READBACK_GPIO_READ_CONF,
    AD5592_REG_READBACK_PWR_DWN_AND_REF_CNTRL,
    AD5592_REG_READBACK_OPEN_DRAIN_CONF,
    AD5592_REG_READBACK_TRI_STAT_CONF,
    AD5592_REG_READBACK_RESERVED,
    AD5592_REG_READBACK_SW_RESET
} ad5592r_reg_readback_t;


/**
 * Liest das Register (reg) aus. Hilfreich um korrekte Konfiguration sicherzustellen.
 * @param reg: Register
 * @return Wert des Registers
 */
uint8_t ad5592r_register_readback(ad5592r_t *obj, ad5592r_reg_readback_t reg);

/**
 * Sendet die Daten (sixteen_bits) und verweist die darauf empfangenen Daten an (rx_data).
 * @param obj: Zeiger auf Objekt
 * @param sixteen_bits: 16-Bit Wort
 * @param rx_data: Zeiger auf Empfangs-Daten
 * @return true wenn erfolgreich, sonst false
 */
bool ad5592r_comm(ad5592r_t *obj, ad5592r_word sixteen_bits, uint8_t *rx_data);

/**
 * Wandelt einen analogen Wert in einen digitalen Wert um und gibt diesen zurück.
 * @param obj: Zeiger auf Objekt
 * @param voltage_mV: Spannung in mV
 * @param double_vref: true, wenn mit doppelter Referenzspannung gerechnet werden soll,
 * false wenn einmal.
 * @return Dezimalwert 0-4095
 */
uint16_t ad5592r_analog_2_digital(ad5592r_t *obj, double voltage_mV, bool double_vref);

/**
 * Wandelt einen digitalen Wert in einen analogen Wert um und gibt diesen zurück.
 * @param obj: Zeiger auf Objekt
 * @param count: Dezimalwert 0-4095
 * @param double_vref: true, wenn mit doppelter Referenzspannung gerechnet werden soll,
 * false wenn einmal.
 * @return Spannung in mV
 */
double ad5592r_digital_2_analog(ad5592r_t *obj, uint16_t sample, bool double_vref);

/**
 * Wandelt einen prozentualen Wert in einen digitalen Wert um und gibt diesen zurück.
 * @param percentage: Prozentangabe
 * @return Dezimalwert 0-4095
 */
uint16_t ad5592r_percentage_2_digital(unsigned int percentage);

/**
 * Wandelt einen prozentualen Wert in einen analogen Wert (mV) um und gibt diesen zurück.
 * Der Rückgabewert ist abhängig von der konfigurierten Eingangspannung (Vdd).
 * @param percentage: Prozentangabe
 * @return Analogwert 0-Vdd in mV
 */
double ad5592r_percentage_2_analog(ad5592r_t *obj, unsigned int percentage);

/**
 * Wandelt den analogen Wert (voltage_mV) in einen Prozentwert um und gibt diesen zurück.
 * Der Rückgabewert ist abhängig von der konfigurierten Eingangspannung (Vdd).
 * @param voltage_mV: Spannung in mV
 * @return Prozentwert
 */
unsigned int ad5592r_analog_2_percentage(ad5592r_t *obj, double voltage_mV);

/**
 * Wandelt das 16 bit Wort (sixteen_bits) in zwei 8 bit Chunks (eight_bits[]).
 * @param eight_bits[]: 8-bit buffer
 * @param sixteen_bits: 16-bit AD5592R Wort
 */
void ad5592r_split_word(uint8_t eight_bits[], ad5592r_word sixteen_bits, size_t arr_size);

/**
 * Wandelt den Wert des Pin-Makros in eine Pin-Nummer um.
 * @param macro: Verwendung von MACRO AD5592R_IO(pin)
 * @return Pin (0-7)
 */
uint8_t ad5592r_macro_2_pin(uint8_t macro);