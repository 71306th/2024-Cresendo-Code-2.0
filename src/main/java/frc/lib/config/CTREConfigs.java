package frc.lib.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public final class CTREConfigs {
  public static CANcoderConfiguration canCoderConfig;

  public CTREConfigs() {

  }

  public static CANcoderConfiguration CTREConfiguration() {
    canCoderConfig = new CANcoderConfiguration();

    /* Swerve CANCoder Configuration */
    canCoderConfig.MagnetSensor.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);
    canCoderConfig.MagnetSensor.withSensorDirection(SensorDirectionValue.Clockwise_Positive);
    return canCoderConfig;
  }
}
