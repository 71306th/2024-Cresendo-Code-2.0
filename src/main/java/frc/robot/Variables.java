// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;

/** Add your docs here. */
public class Variables {
  public static final class DriverControl {
    /* Field Oriented */
    public static boolean fieldOriented = false;

    /* Slow Mode */
    public static boolean slow = false;
  }

  public static final class OperatorControl {
    /* Superstructure State Related */
    public static boolean isAuto = false;
    public static boolean isAmp = false;
    public static boolean isInPlace = false;
    
    /* Manually Controlling Parameters */
    public static double intakeOutput = 0.0;
    public static double tilterOutput = 0.0;

    /* Stop & Stabilizing */
    public static double tilterLockedAngle = 0.0;
    public static boolean tilterManualIsLocked = false;
  }

  public static final class VisionControl {
    /* ID */
    public static double id = -1;

    /* LED Lights */
    public static int LEDLight = 0;

    /* Apriltag Location */
    public static double visionYaw;
    public static Pose3d visionPose3d;
    public static double botToSpeakerDis;

    /* Has Target(Disables Driver Rotating) */
    public static boolean hasTarget = false;
  }
}
