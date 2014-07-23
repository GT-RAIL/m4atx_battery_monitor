Header header
# Voltage measurements (first entry is normally VIN)
float32[] volts_read_value
string[] volts_read_item
# Expected fully charged voltage levels (first is VIN)
float32[] volts_full_value
string[] volts_full_item
#State of charge of the input power
#based off of the input_nominal and battery_dead_voltage parameters
float32 input_soc
# Current readings, A
float32[] current
# PSU temperature readings, deg. C
float32[] temperature
# AC supply connected
bool ac_on
# Battery is charging
bool is_charging
