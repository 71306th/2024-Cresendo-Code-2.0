// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SuperStructure;
import frc.robot.commands.autos.ClaimFloor;
import frc.robot.commands.autos.Idle;
import frc.robot.commands.autos.MoveTo;
import frc.robot.commands.autos.ShootBase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Blue2 extends SequentialCommandGroup {
  public Blue2(Swerve m_Swerve, SuperStructure m_SuperStructure) {
    addCommands(
      new Idle(m_SuperStructure),
      new MoveTo(m_Swerve, new Pose2d(new Translation2d(0, 0), new Rotation2d(0))),
      new ShootBase(m_SuperStructure),
      new Idle(m_SuperStructure),
      new MoveTo(m_Swerve, new Pose2d(new Translation2d(0, 0), new Rotation2d(0))),
      new ClaimFloor(m_SuperStructure),
      new Idle(m_SuperStructure),
      new MoveTo(m_Swerve, new Pose2d(new Translation2d(0, 0), new Rotation2d(0))),
      new ShootBase(m_SuperStructure)
      );
  }
}
