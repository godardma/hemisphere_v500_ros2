std_msgs/Header header

float64 gpstimeofweek  # GPS tow [8 bytes]
uint16  gpsweek        # GPS week [2 bytes]
uint16  numsatstracked # SATS Tracked [2 bytes]
uint16  numsatsused    # SATS Used [2 bytes]
uint8   navmode        # Nav Mode (same as message 1) [1 byte ]
uint8   spare00        # Spare [1 byte ]
float64 latitude       # Latitude degrees, -90..90 [8 bytes]
float64 longitude      # Longitude degrees, -180..180 [8 bytes]
float32 height         # (m), Altitude ellipsoid [4 bytes]
float32 speed          # Horizontal Speed m/s [4 bytes]
float32 vup            # Vertical Velocity +up m/s [4 bytes]
float32 cog            # Course over Ground, degrees [4 bytes]
float32 heading        # Heading (degrees), Zero unless vector[4 bytes]
float32 pitch          # Pitch (degrees), Zero unless vector [4 bytes]
float32 spare01        # Spare [4 bytes]
uint16  ageofdiff      # age of differential, seconds [2 bytes]
# uint16  attitudestatus # Attitude Status, Zero unless vector [2 bytes]
#                        # attitudeStatus: bit {0-3} = sStatus.eYaw
#                        # bit {4-7} = sStatus.ePitch
#                        # bit {8-11} = sStatus.eRoll
#                        # where sStatus can be 0 = INVALID, 1 = GNSS, 2 = Inertial, 3= Magnetic
uint8   yawstatus
uint8   pitchstatus
uint8   rollstatus
uint8   spare02
float32 stdevheading   # Yaw stdev, degrees, 0 unless vector [4 bytes]
float32 stdevpitch     # Pitch stdev, degrees, 0 unless vector[4 bytes]
float32 hrms           # Horizontal RMS [4 bytes]
float32 vrms           # Vertical RMS [4 bytes]
float32 hdop           # Horizontal DOP [4 bytes]
float32 vdop           # Vertical DOP [4 bytes]
float32 tdop           # Time DOP [4 bytes]
float32 covnn          # Covaraince North-North [4 bytes]
float32 covne          # Covaraince North-East [4 bytes]
float32 covnu          # Covaraince North-Up [4 bytes]
float32 covee          # Covaraince East-East [4 bytes]
float32 coveu          # Covaraince East-Up [4 bytes]
float32 covuu          # Covaraince Up-Up [4 bytes]
