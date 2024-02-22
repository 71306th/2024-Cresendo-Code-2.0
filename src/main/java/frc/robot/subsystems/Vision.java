// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Variables;

public class Vision extends SubsystemBase {

  private DoubleSubscriber tv;
  private DoubleSubscriber tx;
  private DoubleSubscriber ty;
  private DoubleSubscriber ta;
  private DoubleSubscriber tid;
  private DoubleArraySubscriber coordinate;
  private IntegerPublisher tled;

  private double valid;
  private double x;
  private double y;
  private double area;
  private double[] coordinateArr;
  private int ledMode;

  NetworkTable table;
  
  public Vision() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tv = table.getDoubleTopic("tv").subscribe(0.0);
    tx = table.getDoubleTopic("tx").subscribe(0.0);
    ty = table.getDoubleTopic("ty").subscribe(0.0);
    ta = table.getDoubleTopic("ta").subscribe(0.0);
    tid = table.getDoubleTopic("tid").subscribe(0.0);
    coordinate = table.getDoubleArrayTopic("targetpose_cameraspace").subscribe(new double[6]); // 開3D模式 2D不行
    tled = table.getIntegerTopic("ledMode").publish();
    ledMode = 0; 
    tled.set(ledMode); // 試試就逝世
  }
  
  @Override
  public void periodic() {

    /* getting values */
    valid = tv.get();
    x = tx.get();
    y = ty.get();
    area = ta.get();
    Variables.VisionControl.id = tid.get();
    coordinateArr = coordinate.get();

    Variables.VisionControl.visionPose3d = new Pose3d(new Translation3d(coordinateArr[0], coordinateArr[1], coordinateArr[2]), new Rotation3d(coordinateArr[5], coordinateArr[3], coordinateArr[4]));
    Variables.VisionControl.visionYaw = x;

    SmartDashboard.putNumber("LimelightVaild", valid);
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("LimelightID", Variables.VisionControl.id);
    SmartDashboard.putNumber("LimelightToTargetX", coordinateArr[0]);
    SmartDashboard.putNumber("LimelightToTargetY", coordinateArr[1]);
    SmartDashboard.putNumber("LimelightToTargetZ", coordinateArr[2]);
    SmartDashboard.putNumber("LimelightToTargetRX", coordinateArr[3]);
    SmartDashboard.putNumber("LimelightToTargetRY", coordinateArr[4]);
    SmartDashboard.putNumber("LimelightToTargetRZ", coordinateArr[5]);
    SmartDashboard.putNumber("LEDStatus", ledMode);
  }

  /* fetches */
  public double getHorizontalDeviation() {
    return x;
  }

  public Pose3d getTagToBot() {
    Translation3d translation3d = new Translation3d(
    coordinateArr[0]+Constants.Vision.cameraToBot.getX(), 
    coordinateArr[1]+Constants.Vision.cameraToBot.getY(), 
    coordinateArr[2]+Constants.Vision.cameraToBot.getZ());

    Rotation3d camera3d = new Rotation3d(coordinateArr[5], coordinateArr[3], coordinateArr[4]);
    Rotation3d rotation3d = camera3d.plus(Constants.Vision.cameraToBot.getRotation());

    return new Pose3d(translation3d, rotation3d);
  }

  /* settings */
  public void setLEDMode(int command){
    ledMode = command;
    tled.set(ledMode);
  }
}