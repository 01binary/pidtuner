# Pins
uint8 LPWMpin   # LPWM pin for driving motor
uint8 RPWMpin   # RPWM pin for driving motor
uint8 ADCpin    # ADC pin for analog/PWM absolute encoder (0 to disable)
uint8 csPin     # Select pin for SPI absolute encoder (0 to disable)
uint8 Apin      # A pin for quadrature encoder
uint8 Bpin      # B pin for quadrature encoder

# Tuning
float64 Kp      # P gain
float64 Ki      # I gain
float64 Di      # D gain
float64 iMin    # Min integral
float64 iMax    # Max integral