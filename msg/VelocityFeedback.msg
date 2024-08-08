float32 command     # PWM command (+LPWM -RPWM)
uint8 LPWM          # LPWM speed
uint8 RPWM          # RPWM speed
float32 absolute    # Absolute encoder position
int32 quadrature    # Quadrature ticks
time time           # Current time
time start          # Last command start time
float32 elapsed     # Time elapsed since last step command
float32 dt          # Time step
uint8 step          # Current step if playing back a step sequence