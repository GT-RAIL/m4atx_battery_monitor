Header header
# Voltage measurements (first entry is normally VIN)
float32[] volts_read
# Expected fully charged voltage levels (first is VIN)
float32[] volts_full
# Current readings, A
float32[] current
# PSU temperature readings, deg. C
float32[] temperature
# AC supply connected
bool ac_on
# Battery is charging
bool is_charging
