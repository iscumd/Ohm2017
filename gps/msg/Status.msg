# status message
Header header

# INS filter status
uint8 MODE_NOT_TRACKING=0
uint8 MODE_ALIGNING=1
uint8 MODE_TRACKING=2

uint8 ins_mode

bool usingGPSHeading
bool gpsCompassActive

bool imu_error
bool magpres_error

# GPS status
uint8 num_sats

uint8 GPS_NO_FIX=0
uint8 GPS_TIME_FIX=1
uint8 GPS_2D_FIX=2
uint8 GPS_3D_FIX=3

uint8 fix

bool gps_error
