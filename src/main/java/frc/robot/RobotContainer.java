// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.TeleopDriver;
import frc.robot.commands.TeleopOperator;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Swerve m_Swerve = new Swerve();
  private final Vision m_Vision = new Vision();
  private final SuperStructure m_SuperStructure = new SuperStructure();
  private final Controller m_Controller = new Controller();

  private final TeleopDriver teleSwerve = new TeleopDriver(m_Swerve, m_Controller);
  private final TeleopOperator teleSuperStructure = new TeleopOperator(m_SuperStructure, m_Controller);

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_Swerve.setDefaultCommand(teleSwerve);
    m_Vision.setDefaultCommand(null);
    m_SuperStructure.setDefaultCommand(teleSuperStructure);

    configureBindings();
  }

  private void configureBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
