Kotlin is great for configuration objects. Learn about the specific function used in this template [here](https://kotlinlang.org/docs/scope-functions.html#apply).

Javadocs for configuration objects can be found below.

| **Device**                             | **Javadocs**                                                                                                                                |
| -------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------- |
| Falcon 500<br>Kraken X44<br>Kraken X60 | [`TalonFXConfiguration`](https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/configs/TalonFXConfiguration.html)         |
| Talon FXS                              | [`TalonFXSConfiguration`](https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/configs/TalonFXSConfiguration.html)       |
| Pigeon 2.0                             | [`Pigeon2Configuration`](https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/configs/Pigeon2Configuration.html)         |
| CANcoder                               | [`CANcoderConfiguration`](https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/configs/CANcoderConfiguration.html)       |
| Everything else                        | [`com.ctre.phoenix6.configs` package](https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/configs/package-summary.html) |

The configuration is not entirely complete; there are many, many configuration objects, and to write out all of them would be challenging to write and read.
___
```kotlin
@JvmStatic // needed for accessing from a Java class
val talonConfig: TalonFXConfiguration = TalonFXConfiguration().apply {
  // each field in the `TalonFXConfiguration` class can be accessed in a configuration file-esque fashion by using the `apply {}` scope function
  // all fields contain their default values in this template, so if you don't know what a field does, you can probably just leave it as is

  /// Most-Used Configuration Objects
  // Current Limits
  CurrentLimits.apply {
    // The amount of current allowed in the motor (motoring and regen current).
    StatorCurrentLimit = 120.0 // double
    // Enable motor stator current limiting.
    StatorCurrentLimitEnable = true
    // The absolute maximum amount of supply current allowed.
    SupplyCurrentLimit = 70.0 // double
    // Enable motor supply current limiting.
    SupplyCurrentLimitEnable = true
    // The amount of supply current allowed after the regular SupplyCurrentLimit is active for longer than SupplyCurrentLowerTime.
    SupplyCurrentLowerLimit = 40.0 // double
    // Reduces supply current to the SupplyCurrentLowerLimit after limiting to SupplyCurrentLimit for this period of time.
    SupplyCurrentLowerTime = 1.0 // double
  }

  // Motor Output
  MotorOutput.apply {
    // When a control request UseTimesync is enabled, this determines the time-sychronized frequency at which control requests are applied.
    ControlTimesyncFreqHz = 0.0 // double
    // Configures the output deadband duty cycle during duty cycle and voltage based control modes.
    DutyCycleNeutralDeadband = 0.0 // double
    // Invert state of the device as seen from the front of the motor.
    Inverted = InvertedValue.CounterClockwise_Positive // InvertedValue
    // The state of the motor controller bridge when output is neutral or disabled.
    NeutralMode = NeutralModeValue.Coast // NeutralModeValue
    // Maximum (forward) output during duty cycle based control modes.
    PeakForwardDutyCycle = 1.0 // double
    // Minimum (reverse) output during duty cycle based control modes.
    PeakReverseDutyCycle = -1.0 // double
  }

  // Feedback
  // Configs that affect the feedback of this motor controller.
  // Includes feedback sensor source, any offsets for the feedback sensor,
  // and various ratios to describe the relationship between the sensor and the mechanism for closed looping.
  Feedback.apply {
    // Device ID of which remote device to use.
    FeedbackRemoteSensorID = 0 // int
    // The offset applied to the absolute integrated rotor sensor.
    FeedbackRotorOffset = 0.0 // double
    // Choose what sensor source is reported via API and used by closed-loop and limit features.
    FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor // FeedbackSensorSourceValue
    // The ratio of motor rotor rotations to remote sensor rotations, where a ratio greater than 1 is a reduction.
    RotorToSensorRatio = 1.0 // double
    // The ratio of sensor rotations to the mechanism's output, where a ratio greater than 1 is a reduction.
    SensorToMechanismRatio = 1.0 // double
    // The configurable time constant of the Kalman velocity filter.
    VelocityFilterTimeConstant = 0.0 // double
  }

  // There are many more fields in the TalonFXConfiguration class, but this is a good starting point.
  // You can find the full list of fields in the TalonFXConfiguration class in the
  // link in the table near the start of this file.
}
```