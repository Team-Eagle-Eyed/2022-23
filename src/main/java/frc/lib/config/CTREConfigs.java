package frc.lib.config;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

import frc.robot.Constants;

public final class CTREConfigs {
  public CANcoderConfiguration swerveCanCoderConfig;

  public CTREConfigs() {
    swerveCanCoderConfig = new CANcoderConfiguration();

    /* Swerve CANCoder Configuration */
    swerveCanCoderConfig.MagnetSensor.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);
    // swerveCanCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert;
    swerveCanCoderConfig.MagnetSensor.withSensorDirection(Constants.Swerve.canCoderInvert);
    /* swerveCanCoderConfig.initializationStrategy =
      SensorInitializationStrategy.BootToAbsolutePosition;
    swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond; */
  }
}
