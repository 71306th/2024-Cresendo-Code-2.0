// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Variables;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.SuperStructure;

public class TeleopOperator extends Command {
  
  private final SuperStructure m_SuperStructure;
  private final Controller m_Controller;

  private final XboxController operator;

  private boolean onepressarmplus = false;
  private boolean onepressarmminus = false;
  private boolean onepressintakeplus = false;
  private boolean onepressintakeminus = false;

  public TeleopOperator(SuperStructure m_SuperStructure2, Controller subsystem2) {
    m_SuperStructure = m_SuperStructure2;
    addRequirements(m_SuperStructure);
    m_Controller = subsystem2;
    operator = m_Controller.getOperatorController();
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(operator.getAButton()) m_SuperStructure.setIntakeLower1Motor();
    if(operator.getXButton()) m_SuperStructure.setIntakeLower2Motor();
    if(operator.getYButton()) m_SuperStructure.setIntakeUpperMotor();
    if(operator.getBButton()) m_SuperStructure.stopIntakeMotor();
    if(operator.getBackButton()) m_SuperStructure.setTilter();
    if(operator.getStartButton()) m_SuperStructure.stopTilter(m_SuperStructure.getTilterAngle());
    if(operator.getLeftBumperPressed() && onepressarmminus == false) {
      Variables.OperatorControl.tilterOutput -= 0.01;
      onepressarmminus = true;
    }
    if(operator.getRightBumperPressed() && onepressarmplus == false) {
      Variables.OperatorControl.tilterOutput += 0.01;
      onepressarmplus = true;
    }
    if(operator.getLeftBumperReleased()) onepressarmminus = false;
    if(operator.getRightBumperReleased()) onepressarmplus = false;
    if(operator.getLeftTriggerAxis() == 1 && onepressintakeminus == false) {
      Variables.OperatorControl.intakeOutput -= 0.05;
      onepressintakeminus = true;
    }
    if(operator.getRightTriggerAxis() == 1 && onepressintakeplus == false) {
      Variables.OperatorControl.intakeOutput += 0.05;
      onepressintakeplus = true;
    }
    if(operator.getLeftTriggerAxis() == 0) onepressintakeminus = false;
    if(operator.getRightTriggerAxis() == 0) onepressintakeplus = false;
  }
}
