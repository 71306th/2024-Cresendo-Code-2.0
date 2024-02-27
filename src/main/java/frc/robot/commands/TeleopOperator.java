// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
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

  private boolean lastIsManualTilter = false;
  private boolean lastIsManualIntake = false;
  private boolean oneTimeIsInPlace = false;
  private boolean oneTimeIsLoaded = false;

  private double leftRumbleTime;
  private double rightRumbleTime;

  public TeleopOperator(SuperStructure m_SuperStructure2, Controller subsystem2) {
    m_SuperStructure = m_SuperStructure2;
    addRequirements(m_SuperStructure);
    m_Controller = subsystem2;
    operator = m_Controller.getOperatorController();
  }

  @Override
  public void initialize() {
    operator.setRumble(RumbleType.kBothRumble, 0);
  }

  @Override
  public void execute() {
    if(operator.getAButton()) m_SuperStructure.setState(InputStates.Auto); lastIsManualIntake = false; lastIsManualTilter = false;
    if(operator.getBButton()) m_SuperStructure.setState(InputStates.Base); lastIsManualIntake = false; lastIsManualTilter = false;
    if(operator.getXButton()) m_SuperStructure.setState(InputStates.Podium); lastIsManualIntake = false; lastIsManualTilter = false;
    if(operator.getYButton()) m_SuperStructure.setState(InputStates.Floor); lastIsManualIntake = false; lastIsManualTilter = false;
    if(operator.getLeftBumper()) m_SuperStructure.setState(InputStates.Amp); lastIsManualIntake = false; lastIsManualTilter = false;
    if(operator.getRightBumper()) m_SuperStructure.setState(InputStates.Idle); lastIsManualIntake = false; lastIsManualTilter = false;
    if(operator.getBackButton()) m_SuperStructure.setState(InputStates.Emergency); lastIsManualIntake = false; lastIsManualTilter = false;
    if(operator.getStartButtonPressed()) m_SuperStructure.setIntakeClaiming(Constants.SuperStructure.intakeClaimSpeed); lastIsManualIntake = false; lastIsManualTilter = false;
    if(operator.getStartButtonReleased()) m_SuperStructure.setIntakeClaiming(0);
    if(operator.getLeftTriggerAxis() >= 0.5) m_SuperStructure.setState(InputStates.ManualTilterRun); lastIsManualTilter = true;
    if(operator.getLeftTriggerAxis() < 0.5 && lastIsManualTilter) m_SuperStructure.setState(InputStates.ManualTilterLock);
    if(operator.getRightTriggerAxis() >= 0.5) m_SuperStructure.setState(InputStates.ManualIntakeRun); lastIsManualIntake = true;
    if(operator.getRightTriggerAxis() < 0.5 && lastIsManualIntake) m_SuperStructure.setState(InputStates.ManualIntakeStop);
    if(operator.getPOV() == 0) Variables.OperatorControl.tilterOutput += 0.01;
    if(operator.getPOV() == 90) Variables.OperatorControl.intakeOutput += 0.01;
    if(operator.getPOV() == 180) Variables.OperatorControl.tilterOutput -= 0.01;
    if(operator.getPOV() == 270) Variables.OperatorControl.intakeOutput -= 0.01;

    // if(m_SuperStructure.isLoaded()){
    //   if(!oneTimeIsLoaded) {
    //     rightRumbleTime = Timer.getFPGATimestamp() + Constants.JoystickConstants.rumbleTime; 
    //     oneTimeIsLoaded = true;
    //   }
    //   if(rightRumbleTime >= Timer.getFPGATimestamp()) operator.setRumble(RumbleType.kRightRumble, 1);
    //   else {
    //     operator.setRumble(RumbleType.kBothRumble, 0); 
    //     oneTimeIsLoaded = false;
    //   }
    // } else
     if (Variables.OperatorControl.isInPlace) {
      if(!oneTimeIsInPlace) {
        leftRumbleTime = Timer.getFPGATimestamp() + Constants.JoystickConstants.rumbleTime; 
        oneTimeIsInPlace = true;
      }
      if(leftRumbleTime >= Timer.getFPGATimestamp()) operator.setRumble(RumbleType.kLeftRumble, 1);
      else {
        operator.setRumble(RumbleType.kBothRumble, 0); 
        oneTimeIsInPlace = false;
      }
    }
  }
}
