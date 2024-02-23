// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Variables;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.InputStates;

public class TeleopOperator extends Command {
  
  private final SuperStructure m_SuperStructure;
  private final Controller m_Controller;

  private final XboxController operator;

  private boolean onePressTilterPlus = false;
  private boolean onePressTilterMinus = false;
  private boolean onePressIntakePlus = false;
  private boolean onePressIntakeMinus = false;

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
    if(operator.getAButton()) m_SuperStructure.setState(InputStates.Auto);
    if(operator.getBButton()) m_SuperStructure.setState(InputStates.Base);
    if(operator.getXButton()) m_SuperStructure.setState(InputStates.Podium);
    if(operator.getYButton()) m_SuperStructure.setState(InputStates.Floor);
    if(operator.getLeftBumper()) m_SuperStructure.setState(InputStates.Amp);
    if(operator.getRightBumper()) m_SuperStructure.setState(InputStates.Idle);
    if(operator.getBackButton()) m_SuperStructure.setState(InputStates.Emergency);
    if(operator.getStartButtonPressed()) m_SuperStructure.setIntakeClaiming(Constants.SuperStructure.intakeClaimSpeed);
    if(operator.getStartButtonReleased()) m_SuperStructure.setIntakeClaiming(0);
    if(operator.getLeftStickButton()) m_SuperStructure.setState(InputStates.ManualTilterLock);
    if(operator.getRightStickButton()) m_SuperStructure.setState(InputStates.ManualIntakeStop);
    if(operator.getPOV() == 0 && onePressTilterPlus == false) {
      Variables.OperatorControl.tilterOutput += 0.05;
      onePressTilterPlus = true;
    }
    if(operator.getPOV() == 180 && onePressTilterMinus == false) {
      Variables.OperatorControl.tilterOutput -= 0.05;
      onePressTilterMinus = true;
    }
    if(operator.getPOV() != 0) onePressTilterMinus = false;
    if(operator.getPOV() != 180) onePressTilterPlus = false;
    if(operator.getPOV() == 90 && onePressIntakePlus == false) {
      Variables.OperatorControl.intakeOutput += 0.05;
      onePressIntakePlus = true;
    }
    if(operator.getPOV() == 270 && onePressIntakeMinus == false) {
      Variables.OperatorControl.intakeOutput -= 0.05;
      onePressIntakeMinus = true;
    }
    if(operator.getPOV() != 90) onePressIntakeMinus = false;
    if(operator.getPOV() != 270) onePressIntakePlus = false;
  }
}
