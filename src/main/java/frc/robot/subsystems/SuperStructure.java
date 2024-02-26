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
  
  // private final I2C.Port i2cPort;
  // private final ColorSensorV3 colorSenser;
  
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

  private double tilterGoalAngle;
  private double intakeShootGoalSpeed;

  private boolean isPID = true;

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

    // i2cPort = I2C.Port.kMXP;
    // colorSenser = new ColorSensorV3(i2cPort);

    limitSwitch = new DigitalInput(Constants.SuperStructure.tilterLimitSwitch);

    tilterPID = new PID(0.03, 0, 0, 0, 0);
    lockPID = new PID(0.05, 0, 0, 0, 0);

    commandState = InputStates.Idle;
    tilterState = TilterStates.idle;
    intakeState = IntakeStates.idle;
  }

  public void setIntakeShooting(double speed) { // 設定轉速
    intakeLowerMaster.set(speed);
  }

  public void setIntakeClaiming(double speed) { // 調好轉速要射(?是要收吧，喔還是把note往shooter推
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
    return -tilterEncoder.getPosition().getValue() * 360;
  }

  public void resetTilterAngle() {
    tilterEncoder.setPosition(0);
  }

  // public boolean isLoaded() {
  //   boolean redTrue = detectedColor.red <= Constants.SuperStructure.noteColorNoShade.red ? detectedColor.red >= Constants.SuperStructure.noteColorInShade.red ? true : false : false;
  //   boolean blueTrue = detectedColor.blue <= Constants.SuperStructure.noteColorNoShade.blue ? detectedColor.blue >= Constants.SuperStructure.noteColorInShade.blue ? true : false : false;
  //   boolean greenTrue = detectedColor.green <= Constants.SuperStructure.noteColorNoShade.green ? detectedColor.green >= Constants.SuperStructure.noteColorInShade.green ? true : false : false;
  //   if(redTrue && blueTrue && greenTrue) return true;
  //   else return false;
  // }

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
        Variables.OperatorControl.isAuto = true;
        Variables.OperatorControl.isAmp = false;
        break;
      case Base:
        tilterState = TilterStates.base;
        intakeState = IntakeStates.base;
        lastTilterState = tilterState;
        lastIntakeState = intakeState;
        Variables.OperatorControl.isAuto = false;
        Variables.OperatorControl.isAmp = false;
      case Podium:
        tilterState = TilterStates.podium;
        intakeState = IntakeStates.podium;
        lastTilterState = tilterState;
        lastIntakeState = intakeState;
        Variables.OperatorControl.isAuto = false;
        Variables.OperatorControl.isAmp = false;
        break;
      case Floor:
        tilterState = TilterStates.floor;
        intakeState = IntakeStates.floor;
        lastTilterState = tilterState;
        lastIntakeState = intakeState;
        Variables.OperatorControl.isAuto = false;
        Variables.OperatorControl.isAmp = false;
        break;
      case Amp:
        tilterState = TilterStates.amp;
        intakeState = IntakeStates.amp;
        lastTilterState = tilterState;
        lastIntakeState = intakeState;
        Variables.OperatorControl.isAuto = false;
        Variables.OperatorControl.isAmp = true;
        break;
      case Idle:
        tilterState = TilterStates.idle;
        intakeState = IntakeStates.idle;
        lastTilterState = tilterState;
        lastIntakeState = intakeState;
        Variables.OperatorControl.isAuto = false;
        Variables.OperatorControl.isAmp = false;
        break;
      case ManualTilterRun:
        tilterState = TilterStates.manualRun;
        intakeState = lastIntakeState;
        lastTilterState = tilterState;
        Variables.OperatorControl.isAuto = false;
        Variables.OperatorControl.isAmp = false;
        break;
      case ManualTilterLock:
        if(lastTilterState == TilterStates.manualRun) tilterState = TilterStates.manualStop;
        else tilterState = lastTilterState;
        intakeState = lastIntakeState;
        lastTilterState = tilterState;
        lastIntakeState = intakeState;
        Variables.OperatorControl.isAuto = false;
        Variables.OperatorControl.isAmp = false;
        break;
      case ManualIntakeRun:
        tilterState = lastTilterState;
        intakeState = IntakeStates.manualRun;
        lastIntakeState = intakeState;
        Variables.OperatorControl.isAuto = false;
        Variables.OperatorControl.isAmp = false;
        break;
      case ManualIntakeStop:
        tilterState = lastTilterState;
        if(lastIntakeState == IntakeStates.manualRun) intakeState = IntakeStates.manualStop;
        else intakeState = lastIntakeState;
        lastTilterState = tilterState;
        lastIntakeState = intakeState;
        Variables.OperatorControl.isAuto = false;
        Variables.OperatorControl.isAmp = false;
        break;
      case Emergency:
        tilterState = TilterStates.emergency;
        intakeState = IntakeStates.emergency;
        lastTilterState = tilterState;
        lastIntakeState = intakeState;
        Variables.OperatorControl.isAuto = false;
        Variables.OperatorControl.isAmp = false;
        break;
    }
  }

  public void updateTilterStates() {
    switch (tilterState) {
      case auto:
        isPID = true;
        if(calculateTilterAngle()!=404) tilterGoalAngle = calculateTilterAngle();
        else tilterGoalAngle = Constants.SuperStructure.tilterIdleAngle;
        break;
      case base:
        isPID = true;
        tilterGoalAngle = Constants.SuperStructure.tilterBaseAngle;
        break;
      case podium:
        isPID = true;
        tilterGoalAngle = Constants.SuperStructure.tilterPodiumAngle;
        break;
      case floor:
        isPID = true;
        tilterGoalAngle = Constants.SuperStructure.tilterFloorAngle;
        break;
      case amp:
        isPID = true;
        tilterGoalAngle = Constants.SuperStructure.tilterAmpAngle;
        break;
      case idle:
        isPID = true;
        tilterGoalAngle = Constants.SuperStructure.tilterIdleAngle;
        break;
      case manualRun:
        isPID = false;
        setTilter(Variables.OperatorControl.tilterOutput);
        break;
      case manualStop:
        isPID = false;
        lockTilter(getTilterAngle());
        break;
      case emergency:
        isPID = true;
        tilterGoalAngle = 0;
        break;
    }
    if(isPID == true) setTilter(MathUtility.clamp(tilterPID.calculate(tilterGoalAngle - getTilterAngle()), -Constants.SuperStructure.tilterAutoMaxSpeed, Constants.SuperStructure.tilterAutoMaxSpeed));
  }

  public void updateIntakeStates() {
    switch (intakeState) {
      case auto:
        if(calculateShootingSpeed()!=404) intakeShootGoalSpeed = calculateShootingSpeed();
        else intakeShootGoalSpeed = Constants.SuperStructure.intakeIdleSpeed;
        break;
      case base:
        intakeShootGoalSpeed = Constants.SuperStructure.intakeBaseSpeed;
        break;
      case podium:
        intakeShootGoalSpeed = Constants.SuperStructure.intakePodiumSpeed;
        break;
      case floor:
        // if(!isLoaded()) setIntakeClaiming(Constants.SuperStructure.intakeClaimSpeed);
        // else setIntakeClaiming(0);
        intakeShootGoalSpeed = 0;
        break;
      case amp:
        intakeShootGoalSpeed = Constants.SuperStructure.intakeAmpSpeed;
        break;
      case idle:
        intakeShootGoalSpeed = Constants.SuperStructure.intakeIdleSpeed;
        break;
      case manualRun:
        intakeShootGoalSpeed = Variables.OperatorControl.intakeOutput;
        break;
      case manualStop:
        intakeShootGoalSpeed = 0;
        break;
      case emergency:
        intakeShootGoalSpeed = 0;
        break;
    }
    setIntakeShooting(intakeShootGoalSpeed);
  }

  @Override
  public void periodic() {
    updateOverallStates();
    updateTilterStates();
    updateIntakeStates();
    if(Variables.OperatorControl.tilterManualIsLocked){
      double currentAngle = getTilterAngle();
      double output = lockPID.calculate(Variables.OperatorControl.tilterLockedAngle - currentAngle);
      tilterMaster.set(output);
    }
    // if(limitSwitch.get()) tilterEncoder.setPosition(0);
    // detectedColor = colorSenser.getColor();
    if((getTilterAngle() <= tilterGoalAngle+1 && getTilterAngle() >= tilterGoalAngle-1) && (
      tilterState == TilterStates.auto || tilterState == TilterStates.base || tilterState == TilterStates.podium || tilterState == TilterStates.floor || tilterState == TilterStates.amp
      )) Variables.OperatorControl.isInPlace = true;
    else Variables.OperatorControl.isInPlace = false;

    SmartDashboard.putNumber("Tilter Angle", getTilterAngle());
    SmartDashboard.putNumber("Intake Speed", intakeShootGoalSpeed);
    SmartDashboard.putNumber("Manual Tilter Percent Output", Variables.OperatorControl.tilterOutput);
    SmartDashboard.putNumber("Manual Tilter Locked Angle", Variables.OperatorControl.tilterLockedAngle);
    SmartDashboard.putNumber("Manual Intake Percent Output", Variables.OperatorControl.intakeOutput);
    SmartDashboard.putString("Tilter State", tilterState.toString());
    SmartDashboard.putString("Intake State", intakeState.toString());
    // SmartDashboard.putNumber("R", detectedColor.red);
    // SmartDashboard.putNumber("G", detectedColor.green);
    // SmartDashboard.putNumber("B", detectedColor.blue);
    // SmartDashboard.putBoolean("Loaded", isLoaded());
    SmartDashboard.putBoolean("Semi-Auto In Place(only auto, base, podium, floor, amp)", Variables.OperatorControl.isInPlace);
  }
}
