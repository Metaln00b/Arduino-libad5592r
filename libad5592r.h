#ifndef libad5592r_h
#define libad5592r_h


/*******************************************************************************
 * Konstanten                                                                  *
 ******************************************************************************/

#define AD5592R_CHANNEL_COUNT       8
#define AD5592R_CHANNEL_MIN         0
#define AD5592R_CHANNEL_MAX         7

#define AD5592R_INT_REF_mV          2500
#define AD5592R_MIN_EXT_REF_mV      1000

#define AD5592R_MIN_VDD_mV          2700 /* Minimale Versorgungsspannung */
#define AD5592R_MAX_VDD_mV          5500 /* Maximale Versorgungsspannung */

#define AD5592R_SAMPLE_CODE_MIN     0
#define AD5592R_SAMPLE_CODE_MAX     4095

#define AD5592R_IO(pin)             (1<<pin)

typedef uint16_t                    ad5592r_word;


/*******************************************************************************
 * Datentypen                                                                  *
 ******************************************************************************/

/* Definitionen der Pinout-Typen */
typedef enum {
    AD5592R_GPIO_DIRECTION_IN,
    AD5592R_GPIO_DIRECTION_OUT
} ad5592r_gpio_direction_t;

/* Definitionen der Pinout-Typen */
typedef enum {
    AD5592R_IO_TYPE_ADC,
    AD5592R_IO_TYPE_DAC,
    AD5592R_IO_TYPE_GPIO
} ad5592r_io_type_t;

/* Definitionen der GPIO-Typen */
typedef enum {
    AD5592R_GPIO_TYPE_PUSH_PULL,
    AD5592R_GPIO_TYPE_OPEN_DRAIN
} ad5592r_gpio_type_t;

/* Definitionen der GPIO-States */
typedef enum {
    AD5592R_GPIO_LOW,
    AD5592R_GPIO_HIGH,
    AD5592R_GPIO_Z
} ad5592r_gpio_state_t;

/* Definitionen von Quellen der Referenzspannung*/
typedef enum {
    AD5592R_VREF_SOURCE_INTERNAL,
    AD5592R_VREF_SOURCE_EXTERNAL
} ad5592r_vref_source_t;

/* Definitionen der LDAC-Modis */
typedef enum {
    /* Daten, die in ein Eingangsregister geschrieben werden, werden sofort in
     * ein DAC-Register kopiert und der DAC Ausgang wird aktualisiert (default). */
    AD5592R_LDAC_MODE_IMMEDIATELY,

    /* Daten, die in ein Eingangsregister geschrieben werden, werden nicht in ein DAC-Register kopiert.
     * Der DAC-Ausgang wird nicht aktualisiert. */
    AD5592R_LDAC_MODE_ADD,

    /* Die Daten in den Eingangsregistern werden in die entsprechenden DAC-Register kopiert.
     * Nach der Übertragung der Daten werden die DAC-Ausgänge gleichzeitig aktualisiert. */
    AD5592R_LDAC_MODE_TRANSFER
} ad5592r_ldac_mode_t;

/* Datentyp für Pin Einstellungen */
typedef struct {
    /* OTHER */
    ad5592r_io_type_t type;

    /* GPIO */
    struct {
        ad5592r_gpio_type_t type;
        ad5592r_gpio_state_t state;
        ad5592r_gpio_direction_t direction;
        bool pull_down;
    } gpio;

    /* DAC */
    struct {
        uint16_t sample;
    } dac;
} ad5592r_io_setting_t;

typedef struct {
    /* Inhalt Datenregister */
    ad5592r_vref_source_t vref_source;

    bool double_vref_adc;
    bool double_vref_dac;

    ad5592r_ldac_mode_t ldac_mode;

    /* Spannung welche an Pin (Vdd) anliegt */
    double supply_voltage_mV;

    /* Spannung welche an Pin (Vref) anliegt, wenn eine externe Referenzspannung verwendet wird */
    double external_ref_voltage_mV;

    ad5592r_io_setting_t io_setting[AD5592R_CHANNEL_COUNT];
} ad5592r_t;


/*******************************************************************************
 * Initialisierung / Deinitialisierung                                         *
 ******************************************************************************/

/**
 * Initialisierung von ad5592r-Objekt
 * @param obj: Zeiger auf Objekt
 */
ad5592r_t *ad5592r_init(ad5592r_t *obj);

/**
 * Zurücksetzen
 * @param obj: Zeiger auf Objekt
 */
void ad5592r_reset(ad5592r_t *obj);


/*******************************************************************************
 * Allgemeine Konfiguration                                                    *
 ******************************************************************************/

/**
 * Speichert die Spannung (voltage_mV) als Eingangspannung, welche maximal 5500mV betragen darf.
 * @param obj: Zeiger auf Objekt
 * @param voltage_mV: Spannung welche an (Vdd) anliegt
 * @return true wenn erfolgreich, sonst false
 */
bool ad5592r_setup_supply_voltage_mV(ad5592r_t *obj, double voltage_mV);

/**
 * Wird die interne Referenzspannung gewählt, wird diese an Pin (Vref) anliegen.
 * Wird die externe Referenzspannung gewählt, sollte eine externe Referenzspannung an Pin (Vref) anliegen.
 * Wichtig: Sobald die interne Referenz gewählt wird, werden 2500mV als
 * Referenzspannung verwendet, unabhängig davon, welche Eingangspannung am AD5592R anliegt!
 * @param obj: Zeiger auf Objekt
 * @param source: Quelle der Referenzspannung (INTERNAL, EXTERNAL)
 */
void ad5592r_setup_source_ref_set(ad5592r_t *obj, ad5592r_vref_source_t source);

/**
 * Speichert die Spannung (voltage_mV) als externe Referenzsspannung.
 * @param obj: Zeiger auf Objekt
 * @param voltage_mV: Referenzspannung (EXTERNAL)
 * @return true wenn erfolgreich, sonst false
 */
bool ad5592r_setup_external_ref_voltage_mV(ad5592r_t *obj, double voltage_mV);

/**
 * Gibt die Quelle der Referenzspannung zurück.
 * @param obj: Zeiger auf Objekt
 * @return Quelle der Referenzspannung (INTERNAL, EXTERNAL)
 */
ad5592r_vref_source_t ad5592r_setup_source_ref_get(ad5592r_t *obj);

/**
 * Setzt den Ausgangswerte-Bereich des ADC und DAC auf die doppelte Referenzspannung (0 - 2xVref).
 * @param obj: Zeiger auf Objekt
 * @param double_vref: Zustand doppelte Referenzspannung (false=0V to Vref;true= 0V to 2xVref)
 */
void ad5592r_setup_double_vref_set(ad5592r_t *obj, bool double_vref);

/**
 * Gibt den Zustand der doppelten Referenzspannung des ADC zurück.
 * @param obj: Zeiger auf Objekt
 * @return true wenn (0 - 2xVref), false wenn (0 - Vref)
 */
bool ad5592r_setup_double_vref_adc_get(ad5592r_t *obj);

/**
 * Gibt den Zustand der doppelten Referenzspannung des DAC zurück.
 * @param obj: Zeiger auf Objekt
 * @return true wenn (0 - 2xVref), false wenn (0 - Vref)
 */
bool ad5592r_setup_double_vref_dac_get(ad5592r_t *obj);


/*******************************************************************************
 * Pin Konfiguration                                                           *
 ******************************************************************************/

/**
 * Legt fest, wie Daten, die in ein Eingangsregister eines DAC geschrieben/behandelt werden.
 * @param obj: Zeiger auf Objekt
 * @param ldac_mode: LDAC-Modus (IMMEDIATELY, ADD, TRANSFER)
 */
void ad5592r_setup_ldac(ad5592r_t *obj, ad5592r_ldac_mode_t ldac_mode);

/**
 * Definiert (pins) als DAC.
 * @param obj: Zeiger auf Objekt
 * @param pins: Verwendung von MACRO AD5592R_IO(pin)
 */
void ad5592r_setup_dac_all(ad5592r_t *obj, uint8_t pins);

/**
 * Definiert (pin) als DAC.
 * @param obj: Zeiger auf Objekt
 * @param pin: Verwendung von MACRO AD5592R_IO(pin)
 */
void ad5592r_setup_dac(ad5592r_t *obj, uint8_t pin);

/**
 * Definiert (pins) als ADC.
 * @param obj: Zeiger auf Objekt
 * @param pins: Verwendung von MACRO AD5592R_IO(pin)
 */
void ad5592r_setup_adc_all(ad5592r_t *obj, uint8_t pins);

/**
 * Definiert (pin) als ADC.
 * @param obj: Zeiger auf Objekt
 * @param pin: Verwendung von MACRO AD5592R_IO(pin)
 */
void ad5592r_setup_adc(ad5592r_t *obj, uint8_t pin);

/**
 * Definiert den Typ (type) von (pins).
 * @param obj: Zeiger auf Objekt
 * @param pins: Verwendung von MACRO AD5592R_IO(pin)
 * @param type: Typ (AD5592R_GPIO_TYPE_PUSH_PULL, AD5592R_GPIO_TYPE_OPEN_DRAIN)
 */
void ad5592r_gpio_setup_type_all(ad5592r_t *obj, uint8_t pins, ad5592r_gpio_type_t type);

/**
 * Definiert den Typ (type) von (pins).
 * @param obj: Zeiger auf Objekt
 * @param pin: Verwendung von MACRO AD5592R_IO(pin)
 * @param type: Typ (AD5592R_GPIO_TYPE_PUSH_PULL, AD5592R_GPIO_TYPE_OPEN_DRAIN)
 */
void ad5592r_gpio_setup_type(ad5592r_t *obj, uint8_t pin, ad5592r_gpio_type_t type);

/**
 * Definiert (pins) als Pulldown-Pins.
 * @param obj: Zeiger auf Objekt
 * @param pins: Verwendung von MACRO AD5592R_IO(pin)
 * @param state: true wenn pulldown, false wenn nicht
 */
void ad5592r_gpio_pulldown_all_set(ad5592r_t *obj, uint8_t pins, bool state);

/**
 * Definiert (pin) als Pulldown-Pins.
 * @param obj: Zeiger auf Objekt
 * @param pin: Verwendung von MACRO AD5592R_IO(pin)
 * @param state: true wenn pulldown, false wenn nicht
 */
void ad5592r_gpio_pulldown_set(ad5592r_t *obj, uint8_t pin, bool state);

/**
 * Definiert die Richtung (direction) von (pins).
 * @param obj: Zeiger auf Objekt
 * @param pins: Verwendung von MACRO AD5592R_IO(pin)
 * @param direction: Richtung (AD5592R_GPIO_DIRECTION_IN, AD5592R_GPIO_DIRECTION_OUT)
 */
void ad5592r_gpio_direction_all_set(ad5592r_t *obj, uint8_t pins, ad5592r_gpio_direction_t direction);

/**
 * Definiert die Richtung (direction) von (pin).
 * @param obj: Zeiger auf Objekt
 * @param pin: Verwendung von MACRO AD5592R_IO(pin)
 * @param direction: Richtung (AD5592R_GPIO_DIRECTION_IN, AD5592R_GPIO_DIRECTION_OUT)
 */
void ad5592r_gpio_direction_set(ad5592r_t *obj, uint8_t pin, ad5592r_gpio_direction_t direction);


/**
 * Setzt den Zustand von (pins) auf (state).
 * FIXME I/O0 nicht als Tri-State setzen, sonst ist es nicht mehr möglich
 * das ADC-Register zuverlässig auszulesen.
 * @param obj: Zeiger auf Objekt
 * @param pins: Verwendung von MACRO AD5592R_IO(pin)
 * @param state: Zustand des Digitalausgangs (LOW, HIGH, Z)
 */
void ad5592r_gpio_state_all_set(ad5592r_t *obj, uint8_t pins, ad5592r_gpio_state_t state);

/**
 * Setzt den Zustand von (pin) auf (state).
 * FIXME I/O0 nicht als Tri-State setzen, sonst ist es nicht mehr möglich
 * das ADC-Register zuverlässig auszulesen.
 * @param obj: Zeiger auf Objekt
 * @param pin: Verwendung von MACRO AD5592R_IO(pin)
 * @param state: Zustand des Digitalausgangs (LOW, HIGH, Z)
 */
void ad5592r_gpio_state_set(ad5592r_t *obj, uint8_t pin, ad5592r_gpio_state_t state);

/**
 * Gibt den eingangsseitigen Zustand von (pins) als Sample zurück.
 * @param obj: Zeiger auf Objekt
 * @param pins: Verwendung von MACRO AD5592R_IO(pin)
 * @return 1 wenn HIGH, 0 wenn LOW
 */
uint8_t ad5592r_gpio_input_state_get(ad5592r_t *obj, uint8_t pins);

/**
 * Gibt den konfigurierten, ausgangseitigen Zustand von (pin) zurück.
 * @param obj: Zeiger auf Objekt
 * @param pin: Verwendung von MACRO AD5592R_IO(pin)
 * @return 1 wenn HIGH, 0 wenn LOW
 */
uint8_t ad5592r_gpio_output_state_get(ad5592r_t *obj, uint8_t pin);

/**
 * Gibt das Sample von (pin) des ADC zurück.
 * @param obj: Zeiger auf Objekt
 * @param pin: Verwendung von MACRO AD5592R_IO(pin)
 * @return Sample des ADC
 */
uint16_t ad5592r_adc_sample_get(ad5592r_t *obj, uint8_t pin);

/**
 * Gibt den analogen Wert (voltage_mV) von (pin) des ADC zurück.
 * @param obj: Zeiger auf Objekt
 * @param pin: Verwendung von MACRO AD5592R_IO(pin)
 * @return Analogwert (voltage_mV) des ADC
 */
double ad5592r_adc_voltage_get_mV(ad5592r_t *obj, uint8_t pin);

/**
 * Gibt den prozentualen Wert der Spannung von (pin) des ADC, in Abhängigkeit der
 * Eingangsspannung (Vdd) zurück.
 * @param obj: Zeiger auf Objekt
 * @param pin: Verwendung von MACRO AD5592R_IO(pin)
 * @return Prozentwert
 */
unsigned int ad5592r_adc_percentage_get(ad5592r_t *obj, uint8_t pin);

/**
 * Setzt an (pin) das Sample (sample).
 * @param obj: Zeiger auf Objekt
 * @param pin: Verwendung von MACRO AD5592R_IO(pin)
 * @param sample: 12-Bit Sample
 * @return true wenn erfolgreich, sonst false
 */
bool ad5592r_dac_sample_set(ad5592r_t *obj, uint8_t pin, uint16_t sample);

/**
 * Legt an (pin) die Spannung (voltage_mV) an.
 * @param obj: Zeiger auf Objekt
 * @param pin: Verwendung von MACRO AD5592R_IO(pin)
 * @param voltage_mV: Spannung in mV
 * @return true wenn erfolgreich, sonst false
 */
bool ad5592r_dac_voltage_set_mV(ad5592r_t *obj, uint8_t pin, double voltage_mV);

/**
 * Legt an (pin) die Spannung in Prozent (percentage) an.
 * Die Spannung wird prozentual zur Eingangspannung angelegt.
 * @param obj: Zeiger auf Objekt
 * @param pin: Verwendung von MACRO AD5592R_IO(pin)
 * @param percantage: Prozentangabe
 * @return true wenn erfolgreich, sonst false
 */
bool ad5592r_dac_percentage_set(ad5592r_t *obj, uint8_t pin, unsigned int percentage);

/**
 * Gibt den zuletzt eingestellten Sample von (pin) des DAC aus dem Shared-Memory zurück.
 * @param obj: Zeiger auf Objekt
 * @param pin: Verwendung von MACRO AD5592R_IO(pin)
 * @return Sample des DAC
 */
uint16_t ad5592r_dac_sample_get(ad5592r_t *obj, uint8_t pin);


/*******************************************************************************
 * Sonstige                                                                    *
 ******************************************************************************/

/**
 * Gibt die Temperatur des internen Temperatursensors vom AD5592R zurück.
 * @param obj: Zeiger auf Objekt
 * @return Temperatur in Grad Celsius
 */
double ad5592r_temperature_get_degC(ad5592r_t *obj);

/**
 * Gibt die Temperatur des internen Temperatursensors vom AD5592R zurück.
 * @param obj: Zeiger auf Objekt
 * @return Temperatur als sample
 */
uint16_t ad5592r_temperature_sample_get(ad5592r_t *obj);

/**
 * Wandelt das ad5592r typische sample der Temperatur in degC um.
 * @param obj: Zeiger auf Objekt
 * @param temperature_sample: Temperatur als sample
 * @return Temperatur in Grad Celsius
 */
double ad5592r_sample_to_temperature_degC(ad5592r_t *obj, uint16_t temperature_sample);
#endif