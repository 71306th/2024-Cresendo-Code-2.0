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
import edu.wpi.first.wpilibj.util.Color;
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
    public static final double rumbleTime = 1;
  }
  
  public static final class Robot {
    public static final String canbus = "GTX7130";
  }

  public static final class Swerve {
    public static final double axisDeadBand = 0.05; // make sure ur robot won't vibrate cuz the joystick gives a input like 0.002 or sth
    public static final int pigeon1 = 13; // advanced gyro
    public static final int pigeon2 = 14;
    public static final int pigeon3 = 15;
    public static final int pigeon4 = 16;
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
    public static final double angleKD = 0;

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
    public static final double slowRegulator = 0.5;
    
    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 10;
      public static final int angleMotorID = 11;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.291016);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 6;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.166992);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Rear Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 9;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.609375);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
    
    /* Rear Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 12;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.025879);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
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
    public static final int intakeLowerMaster = 20;
    public static final int intakeLowerSlave = 21;
    public static final int intakeUpper = 22;
    public static final boolean intakeLowerMasterInverted = true;
    public static final boolean intakeLowerSlaveInverted = false;
    public static final boolean intakeUpperInverted = true;
    public static final Color noteColorInShade = new Color(0.51, 0.157, 0.078);
    public static final Color noteColorNoShade = new Color(1, 0.353, 0.176);

    /* Arm */
    public static final int tilterMaster = 17;
    public static final int tilterSlave = 18;
    public static final int tilterEncoder = 19;
    public static final int tilterLimitSwitch = 1;
    public static final boolean tilterMasterInverted = true;
    public static final boolean tilterSlaveInverted = true;

    /* Reserved Value */
    // Auto
    public static final double tilterAutoMaxSpeed = 0.3;
    public static final double intakeAutoMaxSpeed = 0.6;
    public static final double intakeAutoMinSpeed = 0.4;

    // Base
    public static final double tilterBaseAngle = 0;
    public static final double intakeBaseSpeed = 0;

    // Podium
    public static final double tilterPodiumAngle = 0;
    public static final double intakePodiumSpeed = 0;

    // Floor
    public static final double tilterFloorAngle = 0;
    public static final double intakeClaimSpeed = 0.5;

    // Amplifier
    public static final double tilterAmpAngle = 0;
    public static final double intakeAmpSpeed = 0;

    // Idle
    public static final double tilterIdleAngle = 0;
    public static final double intakeIdleSpeed = 0;
  }
}