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
import frc.robot.commands.autos.Turn;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Blue3 extends SequentialCommandGroup {

  public Blue3() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new Idle(new SuperStructure()),
      new MoveTo(new Swerve(), new Pose2d(new Translation2d(0, 0), new Rotation2d(0))),
      new ShootBase(new SuperStructure()),
      new Idle(new SuperStructure()),
      new MoveTo(new Swerve(), new Pose2d(new Translation2d(0, 0), new Rotation2d(0))),
      new Turn(new Swerve(), 0),
      new MoveTo(new Swerve(), new Pose2d(new Translation2d(0, 0), new Rotation2d(0))),
      new ClaimFloor(new SuperStructure()),
      new Idle(new SuperStructure()),
      new Turn(new Swerve(), 0),
      new MoveTo(new Swerve(), new Pose2d(new Translation2d(0, 0), new Rotation2d(0))),
      new ShootBase(new SuperStructure())
      );
  }
}