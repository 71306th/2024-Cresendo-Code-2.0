// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ChenryLib.PID;
import frc.lib.config.CTREConfigs;
import frc.robot.Constants;
import frc.robot.Variables;

public class SuperStructure extends SubsystemBase {
  
  private final TalonFX tilterMaster;
  private final TalonFX tilterSlave;
  private final CANSparkMax intakeLower1;
  private final CANSparkMax intakeLower2;
  private final TalonSRX intakeUpper;


  private final CANcoder tilterEncoder;
  private final CANcoderConfigurator tilterEncoderConfigurator;

  private PID lockPID;

  public SuperStructure() {
    tilterMaster = new TalonFX(Constants.SuperStructure.tilterMaster, "GTX7130");
    tilterSlave = new TalonFX(Constants.SuperStructure.tilterSlave, "GTX7130");
    
    intakeLower1 = new CANSparkMax(Constants.SuperStructure.intakeLower1, MotorType.kBrushless);
    intakeLower2 = new CANSparkMax(Constants.SuperStructure.intakeLower2, MotorType.kBrushless);
    intakeUpper = new TalonSRX(Constants.SuperStructure.intakeUpper);

    intakeLower1.setIdleMode(IdleMode.kCoast);
    intakeLower2.setIdleMode(IdleMode.kCoast);
    
    intakeLower1.setInverted(Constants.SuperStructure.intakeLower1Inverted);
    intakeLower2.setInverted(Constants.SuperStructure.intakeLower2Inverted);
    intakeUpper.setInverted(Constants.SuperStructure.intakeUpperInverted);
    
    tilterMaster.setInverted(Constants.SuperStructure.tilterMasterInverted);
    tilterSlave.setControl(new Follower(Constants.SuperStructure.tilterMaster, Constants.SuperStructure.tilterSlaveInverted));
    
    tilterEncoder = new CANcoder(Constants.SuperStructure.tilterEncoder, "GTX7130");

    tilterEncoderConfigurator = tilterEncoder.getConfigurator();
    tilterEncoderConfigurator.apply(CTREConfigs.CTREConfiguration());
    tilterEncoder.setPosition(0);
  }

  public void setIntakeLower1Motor() {
    intakeLower1.set(Variables.OperatorControl.intakeOutput);
  }

  public void setIntakeLower2Motor() {
    intakeLower2.set(Variables.OperatorControl.intakeOutput);
  }

  public void setIntakeUpperMotor() {
    intakeUpper.set(TalonSRXControlMode.PercentOutput, 0.3);
  }

  public void stopIntakeMotor() {
    intakeLower1.set(0);
    intakeLower2.set(0);
    intakeUpper.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public void setTilter() {
    Variables.OperatorControl.tilterIsLocked = false;
    tilterMaster.set(Variables.OperatorControl.tilterOutput);
  }

  public void stopTilter(double goal) {
    if(!Variables.OperatorControl.tilterIsLocked) { 
      Variables.OperatorControl.tilterLockedAngle = goal;
      Variables.OperatorControl.tilterIsLocked = true;
    }
  }

  public double getTilterAngle() {
    return tilterEncoder.getPosition().getValue() * 360;
  }

  public void resetTilterAngle() {
    tilterEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Percent Output", Variables.OperatorControl.tilterOutput);
    SmartDashboard.putNumber("Arm Angle", getTilterAngle());
    SmartDashboard.putNumber("Locked Angle", Variables.OperatorControl.tilterLockedAngle);
    SmartDashboard.putNumber("Intake Percent Output", Variables.OperatorControl.intakeOutput);
    if(Variables.OperatorControl.tilterIsLocked){
      double currentAngle = getTilterAngle();
      lockPID = new PID(0.05, 0, 0, 0, 0);
      double output = lockPID.calculate(Variables.OperatorControl.tilterLockedAngle - currentAngle);
      tilterMaster.set(output);
    }
  }
}
