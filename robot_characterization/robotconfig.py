{
    # Class names of motor controllers used.
    # Options:
    # 'Spark'
    # 'Victor'
    # 'VictorSP'
    # 'PWMTalonSRX'
    # 'PWMVictorSPX'
    # 'WPI_TalonSRX'
    # 'WPI_VictorSPX'
    "rightControllerTypes": ["VictorSP", "VictorSP", "VictorSP"],
    "leftControllerTypes": ["VictorSP", "VictorSP", "VictorSP"],
    # Ports for the left-side motors
    "leftMotorPorts": [0, 1, 2],
    # Ports for the right-side motors
    "rightMotorPorts": [3, 4, 5],
    # Inversions for the left-side motors
    "leftMotorsInverted": [False, False, False],
    # Inversions for the right side motors
    "rightMotorsInverted": [False, False, False],
    # Wheel diameter (in units of your choice - will dictate units of analysis)
    "wheelDiameter": 6.125,
    # If your robot has only one encoder, set all right encoder fields to `None`
    # Encoder edges-per-revolution (*NOT* cycles per revolution!)
    # This value should be the edges per revolution *of the wheels*, and so
    # should take into account gearing between the encoder and the wheels
    # encoder = 256, wheel is on 50 tooth gear, encoder on 14 tooth gear
    "encoderEPR": 3657.14,
    # Ports for the left-side encoder
    "leftEncoderPorts": [2, 3],
    # Ports for the right-side encoder
    "rightEncoderPorts": [0, 1],
    # Whether the left encoder is inverted
    "leftEncoderInverted": True,
    # Whether the right encoder is inverted:
    "rightEncoderInverted": False,
    # Your gyro type (one of "NavX", "Pigeon", "ADXRS450", "AnalogGyro", or "None")
    "gyroType": "ADXRS450",
    # Whatever you put into the constructor of your gyro
    # Could be:
    # "SPI.Port.kMXP" (MXP SPI port for NavX or ADXRS450),
    # "SerialPort.Port.kMXP" (MXP Serial port for NavX),
    # "I2C.Port.kOnboard" (Onboard I2C port for NavX),
    # "0" (Pigeon CAN ID or AnalogGyro channel),
    # "new WPI_TalonSRX(3)" (Pigeon on a Talon SRX),
    # "" (NavX using default SPI, ADXRS450 using onboard CS0, or no gyro)
    "gyroPort": "SPI.Port.kOnboardCS0",
}






