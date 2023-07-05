std_msgs/Header header

uint8   ageofdiff     # age of differential, seconds (255 max)
uint8   numofsats     # number of satellites used (12 max)
uint16  gpsweek       # GPS week
float64 gpstimeofweek # GPS tow
float64 latitude      # Latitude degrees, -90..90
float64 longitude     # Longitude degrees, -180..180
float32 height        # (m), Altitude ellipsoid
float32 vnorth        # Velocity north m/s
float32 veast         # Velocity east m/s
float32 vup           # Velocity up m/s
float32 stddevresid   # (m), Standard Deviation of Residuals
uint16  navmode
uint16  ageofdifflong # age of diff using 16 bits (WHY?)

