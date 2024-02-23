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
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ChenryLib.MathUtility;
import frc.ChenryLib.PID;
import frc.lib.config.CTREConfigs;
import frc.robot.Constants;
import frc.robot.Variables;

public class SuperStructure extends SubsystemBase {
  
  private final TalonFX tilterMaster;
  private final TalonFX tilterSlave;
  private final CANSparkMax intakeLowerMaster;
  private final CANSparkMax intakeLowerSlave;
  private final TalonSRX intakeUpper;
  
  private final CANcoder tilterEncoder;
  private final CANcoderConfigurator tilterEncoderConfigurator;
  
  private final I2C.Port i2cPort;
  private final ColorSensorV3 colorSenser;
  
  private final DigitalInput limitSwitch;
  
  private Color detectedColor;

  private PID tilterPID;
  private PID lockPID;

  public static enum TilterStates {
    auto,
    podium,
    base,
    floor,
    amp,
    manualRun,
    manualStop,
    emergency,
    idle
  }

  public static enum IntakeStates {
    auto,
    podium,
    base,
    floor,
    amp,
    manualRun,
    manualStop,
    emergency,
    idle
  }

  public static enum InputStates {
    Auto,
    Base,
    Podium,
    Floor,
    Amp,
    ManualTilterRun,
    ManualTilterLock,
    ManualIntakeRun,
    ManualIntakeStop,
    Emergency,
    Idle
  }

  private InputStates commandState;
  private TilterStates tilterState;
  private TilterStates lastTilterState;
  private IntakeStates intakeState;
  private IntakeStates lastIntakeState;

  public SuperStructure() {
    tilterMaster = new TalonFX(Constants.SuperStructure.tilterMaster, "GTX7130");
    tilterSlave = new TalonFX(Constants.SuperStructure.tilterSlave, "GTX7130");
    
    intakeLowerMaster = new CANSparkMax(Constants.SuperStructure.intakeLowerMaster, MotorType.kBrushless);
    intakeLowerSlave = new CANSparkMax(Constants.SuperStructure.intakeLowerSlave, MotorType.kBrushless);
    intakeUpper = new TalonSRX(Constants.SuperStructure.intakeUpper);

    intakeLowerMaster.setIdleMode(IdleMode.kCoast);
    intakeLowerSlave.setIdleMode(IdleMode.kCoast);
    
    intakeLowerMaster.setInverted(Constants.SuperStructure.intakeLowerMasterInverted);
    intakeLowerSlave.follow(intakeLowerMaster, Constants.SuperStructure.intakeLowerSlaveInverted);
    intakeUpper.setInverted(Constants.SuperStructure.intakeUpperInverted);
    
    tilterMaster.setInverted(Constants.SuperStructure.tilterMasterInverted);
    tilterSlave.setControl(new Follower(Constants.SuperStructure.tilterMaster, Constants.SuperStructure.tilterSlaveInverted));
    
    tilterEncoder = new CANcoder(Constants.SuperStructure.tilterEncoder, "GTX7130");

    tilterEncoderConfigurator = tilterEncoder.getConfigurator();
    tilterEncoderConfigurator.apply(CTREConfigs.CTREConfiguration());
    tilterEncoder.setPosition(0);

    i2cPort = I2C.Port.kOnboard;
    colorSenser = new ColorSensorV3(i2cPort);

    limitSwitch = new DigitalInput(Constants.SuperStructure.tilterLimitSwitch);

    tilterPID = new PID(0.03, 0, 0, 0, 0);
    lockPID = new PID(0.05, 0, 0, 0, 0);

    commandState = InputStates.Idle;
    tilterState = TilterStates.idle;
    intakeState = IntakeStates.idle;
  }

  public void setIntakeShooting(double speed) {
    intakeLowerMaster.set(speed);
  }

  public void setIntakeClaiming(double speed) {
    intakeUpper.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void stopIntakeRunning() {
    intakeLowerMaster.set(0);
    intakeLowerSlave.set(0);
    intakeUpper.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public void setTilter(double speed) {
    Variables.OperatorControl.tilterManualIsLocked = false;
    tilterMaster.set(speed);
  }

  public void lockTilter(double goal) {
    if(!Variables.OperatorControl.tilterManualIsLocked) { 
      Variables.OperatorControl.tilterLockedAngle = goal;
      Variables.OperatorControl.tilterManualIsLocked = true;
    }
  }

  public void stopTilter() {
    Variables.OperatorControl.tilterManualIsLocked = false;
    tilterMaster.set(0);
  }

  public double getTilterAngle() {
    return tilterEncoder.getPosition().getValue() * 360;
  }

  public void resetTilterAngle() {
    tilterEncoder.setPosition(0);
  }

  public boolean isLoaded() {
    boolean redTrue = detectedColor.red <= Constants.SuperStructure.noteColorNoShade.red ? detectedColor.red >= Constants.SuperStructure.noteColorInShade.red ? true : false : false;
    boolean blueTrue = detectedColor.blue <= Constants.SuperStructure.noteColorNoShade.blue ? detectedColor.blue >= Constants.SuperStructure.noteColorInShade.blue ? true : false : false;
    boolean greenTrue = detectedColor.green <= Constants.SuperStructure.noteColorNoShade.green ? detectedColor.green >= Constants.SuperStructure.noteColorInShade.green ? true : false : false;
    if(redTrue && blueTrue && greenTrue) return true;
    else return false;
  }

  public InputStates getState() {
    return commandState;
  }

  public void setState(InputStates state) {
    commandState = state;
  }

  public double calculateTilterAngle() {
    return 404;
  }

  public double calculateShootingSpeed() {
    return 404;
  }

  public void updateOverallStates() {
    switch(commandState){
      case Auto:
        tilterState = TilterStates.auto;
        intakeState = IntakeStates.auto;
        lastTilterState = tilterState;
        lastIntakeState = intakeState;
        break;
      case Base:
        tilterState = TilterStates.base;
        intakeState = IntakeStates.base;
        lastTilterState = tilterState;
        lastIntakeState = intakeState;
      case Podium:
        tilterState = TilterStates.podium;
        intakeState = IntakeStates.podium;
        lastTilterState = tilterState;
        lastIntakeState = intakeState;
        break;
      case Floor:
        tilterState = TilterStates.floor;
        intakeState = IntakeStates.floor;
        lastTilterState = tilterState;
        lastIntakeState = intakeState;
        break;
      case Amp:
        tilterState = TilterStates.amp;
        intakeState = IntakeStates.amp;
        lastTilterState = tilterState;
        lastIntakeState = intakeState;
        break;
      case Idle:
        tilterState = TilterStates.idle;
        intakeState = IntakeStates.idle;
        lastTilterState = tilterState;
        lastIntakeState = intakeState;
        break;
      case ManualTilterRun:
        tilterState = TilterStates.manualRun;
        intakeState = lastIntakeState;
        lastTilterState = tilterState;
        break;
      case ManualTilterLock:
        if(lastTilterState == TilterStates.manualRun) tilterState = TilterStates.manualStop;
        else tilterState = lastTilterState;
        intakeState = lastIntakeState;
        break;
      case ManualIntakeRun:
        tilterState = lastTilterState;
        intakeState = IntakeStates.manualRun;
        lastIntakeState = intakeState;
        break;
      case ManualIntakeStop:
      tilterState = lastTilterState;
      if(lastIntakeState == IntakeStates.manualRun) intakeState = IntakeStates.manualStop;
      else intakeState = lastIntakeState;
        break;
      case Emergency:
        tilterState = TilterStates.emergency;
        intakeState = IntakeStates.emergency;
        lastTilterState = tilterState;
        lastIntakeState = intakeState;
        break;
    }
  }

  public void updateTilterStates() {
    switch (tilterState) {
      case auto:
        if(calculateTilterAngle()!=404) setTilter(MathUtility.clamp(tilterPID.calculate(calculateTilterAngle() - getTilterAngle()), -Constants.SuperStructure.tilterAutoMaxSpeed, Constants.SuperStructure.tilterAutoMaxSpeed));
        else setTilter(MathUtility.clamp(tilterPID.calculate(Constants.SuperStructure.tilterIdleAngle - getTilterAngle()), -Constants.SuperStructure.tilterAutoMaxSpeed, Constants.SuperStructure.tilterAutoMaxSpeed));
        break;
      case base:
        setTilter(MathUtility.clamp(tilterPID.calculate(Constants.SuperStructure.tilterBaseAngle - getTilterAngle()), -Constants.SuperStructure.tilterAutoMaxSpeed, Constants.SuperStructure.tilterAutoMaxSpeed));
        break;
      case podium:
        setTilter(MathUtility.clamp(tilterPID.calculate(Constants.SuperStructure.tilterPodiumAngle - getTilterAngle()), -Constants.SuperStructure.tilterAutoMaxSpeed, Constants.SuperStructure.tilterAutoMaxSpeed));
        break;
      case floor:
        setTilter(MathUtility.clamp(tilterPID.calculate(Constants.SuperStructure.tilterBaseAngle - getTilterAngle()), -Constants.SuperStructure.tilterAutoMaxSpeed, Constants.SuperStructure.tilterAutoMaxSpeed));
        break;
      case amp:
        setTilter(MathUtility.clamp(tilterPID.calculate(Constants.SuperStructure.tilterAmpAngle - getTilterAngle()), -Constants.SuperStructure.tilterAutoMaxSpeed, Constants.SuperStructure.tilterAutoMaxSpeed));
        break;
      case idle:
        setTilter(MathUtility.clamp(tilterPID.calculate(Constants.SuperStructure.tilterIdleAngle - getTilterAngle()), -Constants.SuperStructure.tilterAutoMaxSpeed, Constants.SuperStructure.tilterAutoMaxSpeed));
        break;
      case manualRun:
        setTilter(Variables.OperatorControl.tilterOutput);
        break;
      case manualStop:
        lockTilter(getTilterAngle());
        break;
      case emergency:
        setTilter(0);
        break;
    }
  }

  public void updateIntakeStates() {
    switch (intakeState) {
      case auto:
        if(calculateShootingSpeed()!=404) setIntakeShooting(MathUtility.clamp(calculateShootingSpeed(), Constants.SuperStructure.intakeAutoMinSpeed, Constants.SuperStructure.intakeAutoMaxSpeed));
        else setIntakeShooting(Constants.SuperStructure.intakeIdleSpeed);
        break;
      case base:
        setIntakeShooting(Constants.SuperStructure.intakeBaseSpeed);
        break;
      case podium:
        setIntakeShooting(Constants.SuperStructure.intakePodiumSpeed);
        break;
      case floor:
        if(!isLoaded()) setIntakeClaiming(Constants.SuperStructure.intakeClaimSpeed);
        else setIntakeClaiming(0);
        break;
      case amp:
        setIntakeShooting(Constants.SuperStructure.intakeAmpSpeed);
        break;
      case idle:
        setIntakeShooting(Constants.SuperStructure.intakeIdleSpeed);
        break;
      case manualRun:
        setIntakeShooting(Variables.OperatorControl.intakeOutput);
        break;
      case manualStop:
        setIntakeShooting(0);
        break;
      case emergency:
        setIntakeShooting(0);
        break;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Percent Output", Variables.OperatorControl.tilterOutput);
    SmartDashboard.putNumber("Arm Angle", getTilterAngle());
    SmartDashboard.putNumber("Locked Angle", Variables.OperatorControl.tilterLockedAngle);
    SmartDashboard.putNumber("Intake Percent Output", Variables.OperatorControl.intakeOutput);
    if(Variables.OperatorControl.tilterManualIsLocked){
      double currentAngle = getTilterAngle();
      double output = lockPID.calculate(Variables.OperatorControl.tilterLockedAngle - currentAngle);
      tilterMaster.set(output);
    }
    if(limitSwitch.get()) tilterEncoder.setPosition(0);

    detectedColor = colorSenser.getColor();
    SmartDashboard.putNumber("R", detectedColor.red);
    SmartDashboard.putNumber("G", detectedColor.green);
    SmartDashboard.putNumber("B", detectedColor.blue);
    SmartDashboard.putBoolean("Loaded", isLoaded());
  }
}
