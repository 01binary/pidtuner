float32 command     # PWM command (+LPWM -RPWM)
float32 absolute    # Absolute encoder position
int32 quadrature    # Quadrature ticks
time time           # Current time
time start          # Last command start time
float32 elapsed     # Time elapsed since last step command
float32 dt          # Time step
uint8 step          # Current step if playing back a step sequence
uint8 estop         # Whether currently stopped
uint8 mode          # Current mode (velocity 0, position 1, step 2)
float32 volts       # Motor volts
float32 amps        # Motor amps
