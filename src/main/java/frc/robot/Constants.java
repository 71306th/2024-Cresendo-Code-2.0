// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class JoystickConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static final class Swerve {
    public static final double axisDeadBand = 0.05; // make sure ur robot won't vibrate cuz the joystick gives a input like 0.002 or sth
    public static final int pigeon1 = 5; // advanced gyro
    public static final int pigeon2 = 6;
    public static final int pigeon3 = 7;
    public static final int pigeon4 = 8;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = 0.583; // meters, length between two side's wheels, need to adjust
    public static final double wheelBase = 0.583; // meters, length between same side's wheels, need to adjust
    public static final double wheelDiameter = Units.inchesToMeters(4.0); // need to adjust
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;  // open loop means no feedback(PID), closed loop vise versa, not used actually

    public static final double driveGearRatio = (6.12244897959 / 1.0); // 6.12:1 (6.12244897959), for MK4i(L3)
    public static final double angleGearRatio = (150.0 / 7.0 / 1.0); // 150/7 : 1, for MK4i(all)

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)); // locating Swerve's positions, notice the sequences(first is 0, second is 1, etc.)

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0; // setting the nominal voltage(won't really follow anyway)

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 30; //20
    public static final int driveContinuousCurrentLimit = 40; //80, limiting the amps so Neo won't brake

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKD = 0.0; // maybe need to adjust

    /* Drive Motor PID Values */
    public static final double driveKP = 0.12;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0025;
    public static final double driveKFF = 0.0; // maybe need to adjust

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27; // feedforward, maybe need to adjust

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = wheelCircumference / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio; // like constants in physics

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.4; // meters per second
    public static final double maxAngularVelocity = 13.5; // meters per second

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake; // whether u want to let neo stop slowly individually(coast) or fiercely wholely(brake)

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = false; // yeah invert the motor
    
    /* Slow Mode */
    public static double slowRegulator = 0.5;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 12;
      public static final int canCoderID = 1;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.302978);

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 21;
      public static final int angleMotorID = 22;
      public static final int canCoderID = 2;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.720458);

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 31;
      public static final int angleMotorID = 32;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.969482);

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }
    
    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 41;
      public static final int angleMotorID = 42;
      public static final int canCoderID = 4;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.607666);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }
  }

  public static final class Vision {
    public static final double cameraRoll = 35.5; // degrees
    public static final Pose3d cameraToBot = new Pose3d(new Translation3d(0, 0.1725, -0.1778), new Rotation3d(cameraRoll, 0, 0));
    public static final Pose3d cameraToTilter = new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
    public static final double centerIDToSpeakerZ = 0.795;
    public static final double rightIDToCenterY = 0.565;
  }

  public static final class SuperStructure {
    /* Intake */
    public static final int intakeLower1 = 1;
    public static final int intakeLower2 = 2;
    public static final int intakeUpper = 3;
    public static final boolean intakeLower1Inverted = true;
    public static final boolean intakeLower2Inverted = true;
    public static final boolean intakeUpperInverted = true;

    /* Arm */
    public static final int tilterMaster = 4;
    public static final int tilterSlave = 5;
    public static final int tilterEncoder = 6;
    public static final boolean tilterMasterInverted = true;
    public static final boolean tilterSlaveInverted = true;
  }
}