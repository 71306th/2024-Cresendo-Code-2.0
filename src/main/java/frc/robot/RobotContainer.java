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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.autos.modes.Blue1;
import frc.robot.commands.autos.modes.Blue2;
import frc.robot.commands.autos.modes.Blue3;
import frc.robot.commands.autos.modes.Red1;
import frc.robot.commands.autos.modes.Red2;
import frc.robot.commands.autos.modes.Red3;

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

  private final Blue1 B1 = new Blue1();
  private final Blue2 B2 = new Blue2();
  private final Blue3 B3 = new Blue3();
  private final Red1 R1 = new Red1();
  private final Red2 R2 = new Red2();
  private final Red3 R3 = new Red3();


  SendableChooser<Command> m_Chooser = new SendableChooser<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_Swerve.setDefaultCommand(teleSwerve);
    m_Vision.setDefaultCommand(null);
    m_SuperStructure.setDefaultCommand(teleSuperStructure);

    configureBindings();
  }

  private void configureBindings() {

    m_Chooser.setDefaultOption("Blue 1", B1); // close to amp
    m_Chooser.addOption("Blue 2 meters", B2); // middle
    m_Chooser.addOption("Blue 3", B3); // close to podium
    m_Chooser.setDefaultOption("Red 1", R1); // close to amp
    m_Chooser.addOption("Red 2", R2); //  middle
    m_Chooser.addOption("Red 3", R3); // close to podium

    SmartDashboard.putData("Choosing", m_Chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_Chooser.getSelected();
  }
}
