#define kpa_atm_conversion 0.00986923267
unsigned long kpa_to_atm() {
    double conversion = 40000 * kpa_atm_conversion;
    return adcPressure / conversion;
}
