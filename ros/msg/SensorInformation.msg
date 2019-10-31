Header header

uint64 handle
uint64 serial_number
string model_name
uint16 model
string firmware_version

float32 last_reported_temperature       
float32 last_reported_humidity          
float32 last_reported_age              

float32 measurement_period  

int64 ptp_ts  

uint8 gps_ts_year  
uint8 gps_ts_month  
uint8 gps_ts_day    
uint8 gps_ts_hour   
uint8 gps_ts_min    
uint8 gps_ts_sec    

uint8 return_count
uint8 segment_count  

bool is_mocked
bool is_pps_connected
bool is_nmea_connected
bool is_ptp_connected
bool is_calibrated
bool is_over_heated

int16 cepton_sdk_version
uint8[] data # `cepton_sdk::SensorInformation` bytes