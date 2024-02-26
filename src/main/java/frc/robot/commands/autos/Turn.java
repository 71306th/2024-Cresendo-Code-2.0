// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.ChenryLib.PID;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class Turn extends InstantCommand { // actually no need

  private Swerve m_Swerve = new Swerve();
  private final PID turnPID = new PID(0, 0, 0, 0, 0);
  private double goalAngle;
  private double currentAngle; 
  private double error;

  public Turn(Swerve m_Swerve, double goalAngle) {
    // Use addRequirements() here to declare subsystem dependencies.\
    this.m_Swerve = m_Swerve;
    addRequirements(m_Swerve);
    this.goalAngle = goalAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Swerve.drive(null, goalAngle, isFinished(), isScheduled());
    currentAngle = m_Swerve.getYaw().getDegrees();
    error = goalAngle - currentAngle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = m_Swerve.getYaw().getDegrees();
    m_Swerve.drive(new Translation2d(0, 0), 
    turnPID.calculate(goalAngle - currentAngle) * Constants.Swerve.maxAngularVelocity, 
    true,
    true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(error)<1.5) {return true;
    } else return false;
  }
}
