# Pins
uint8 LPWMpin           # LPWM pin for driving motor
uint8 RPWMpin           # RPWM pin for driving motor
uint8 ADCpin            # ADC pin for analog/PWM absolute encoder (0 to disable)
uint8 csPin             # Select pin for SPI absolute encoder (0 to disable)
uint8 Apin              # A pin for quadrature encoder
uint8 Bpin              # B pin for quadrature encoder

# Limits
float32 pwmMin            # Min PWM value
float32 pwmMax            # Max PWM value
float32 absoluteMin       # Min absolute encoder value
float32 absoluteMax       # Max absolute encoder value

# Inversions
bool pwmInvert          # Whether to invert PWM commands
bool absoluteInvert     # Whether to invert absolute encoder readings
bool quadratureInvert   # Whether to invert quadrature encoder readings

# Tuning
float32 Kp              # P gain
float32 Ki              # I gain
float32 Kd              # D gain
float32 iMin            # Min integral
float32 iMax            # Max integral