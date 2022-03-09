// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import frc.robot.Constants.IntestineConstants;
import frc.robot.Constants.ShooterConstants;

/** Add your docs here. */
public class Shooter extends SubsystemBase {
  ControlType VelocityControlMode = CANSparkMax.ControlType.kSmartVelocity;
  private Timer m_timer = new Timer();

  //Shooter state variables
  private double shooterRPMOpPoint = ShooterConstants.kShootHighRPM;
  private double hoodRPMOpPoint = ShooterConstants.kHoodWheelHighRPM;
  private boolean shooterIsRunning = false;
  
  //Motor config
  private CANSparkMax shooterFeederMotor;
  private CANSparkMax shooterMotor;
  private CANSparkMax shooterSecondaryMotor;
  private CANSparkMax hoodWheelMotor;
  private CANSparkMax turretMotor;
  private CANSparkMax hoodMotor;

  private SparkMaxPIDController shooterPIDController;
  private RelativeEncoder shooterEncoder;

  private SparkMaxPIDController hoodPIDController;
  private RelativeEncoder hoodEncoder;

  private SparkMaxPIDController hoodWheelPIDController;
  private RelativeEncoder hoodWheelEncoder;

  private SparkMaxPIDController turretPIDController;
  private RelativeEncoder turretEncoder;

  private double lastShooterSetRPM = 0;
  private double lastHoodWheelSetRPM = 0;

  public Shooter(){
    int smartMotionSlot = 0;
    
    // Shooter motor configuration
    shooterFeederMotor = new CANSparkMax(ShooterConstants.kShooterFeederMotorPort, MotorType.kBrushless);
    shooterFeederMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    
    shooterMotor = new CANSparkMax(ShooterConstants.kShooterMotorPort, MotorType.kBrushless);
    shooterMotor.restoreFactoryDefaults();
    shooterMotor.setInverted(true);
    shooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    shooterPIDController = shooterMotor.getPIDController();
    shooterEncoder = shooterMotor.getEncoder();

    if (ShooterConstants.hasSecondary) {
      shooterSecondaryMotor = new CANSparkMax(ShooterConstants.kShooterSecondaryMotorPort, MotorType.kBrushless);
      shooterSecondaryMotor.restoreFactoryDefaults();
      shooterSecondaryMotor.setInverted(false);
      shooterSecondaryMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    // set Shooter PID coefficients
    shooterPIDController.setP(ShooterConstants.Shooter.kP);
    shooterPIDController.setI(ShooterConstants.Shooter.kI);
    shooterPIDController.setD(ShooterConstants.Shooter.kD);
    shooterPIDController.setIZone(ShooterConstants.Shooter.kIz);
    shooterPIDController.setFF(ShooterConstants.Shooter.kFF);
    shooterPIDController.setOutputRange(ShooterConstants.Shooter.kMinOutput, ShooterConstants.Shooter.kMaxOutput);
    
    shooterPIDController.setSmartMotionMaxVelocity(ShooterConstants.Shooter.kmaxRPM, smartMotionSlot);
    shooterPIDController.setSmartMotionMinOutputVelocity(50, smartMotionSlot);
    shooterPIDController.setSmartMotionMaxAccel(3000, smartMotionSlot);
    shooterPIDController.setSmartMotionAllowedClosedLoopError(0, smartMotionSlot);
    shooterPIDController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, smartMotionSlot);

    // Hood motor configuration
    hoodMotor = new CANSparkMax(ShooterConstants.kHoodMotorPort, MotorType.kBrushless);
    hoodMotor.restoreFactoryDefaults();
    hoodMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    hoodPIDController = hoodMotor.getPIDController();
    hoodEncoder = hoodMotor.getEncoder();

    // set Hood PID coefficients
    hoodPIDController.setP(ShooterConstants.Hood.kP);
    hoodPIDController.setI(ShooterConstants.Hood.kI);
    hoodPIDController.setD(ShooterConstants.Hood.kD);
    hoodPIDController.setIZone(ShooterConstants.Hood.kIz);
    hoodPIDController.setFF(ShooterConstants.Hood.kFF);
    hoodPIDController.setOutputRange(ShooterConstants.Hood.kMinOutput, ShooterConstants.Hood.kMaxOutput);

    hoodPIDController.setSmartMotionMaxVelocity(ShooterConstants.Hood.kmaxRPM, smartMotionSlot);
    hoodPIDController.setSmartMotionMinOutputVelocity(50, smartMotionSlot);
    hoodPIDController.setSmartMotionMaxAccel(3000, smartMotionSlot);
    hoodPIDController.setSmartMotionAllowedClosedLoopError(0, smartMotionSlot);
    hoodPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, smartMotionSlot);

    // Hood Wheel motor configuration
    hoodWheelMotor = new CANSparkMax(ShooterConstants.kHoodWheelMotorPort, MotorType.kBrushless);
    hoodWheelMotor.restoreFactoryDefaults();
    hoodWheelMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    hoodWheelPIDController = hoodWheelMotor.getPIDController();
    hoodWheelEncoder = hoodWheelMotor.getEncoder();

    // set Hood Wheel PID coefficients
    hoodWheelPIDController.setP(ShooterConstants.kHoodWheelP);
    hoodWheelPIDController.setI(ShooterConstants.kHoodWheelI);
    hoodWheelPIDController.setD(ShooterConstants.kHoodWheelD);
    hoodWheelPIDController.setIZone(ShooterConstants.kHoodWheelIz);
    hoodWheelPIDController.setFF(ShooterConstants.kHoodWheelFF);
    hoodWheelPIDController.setOutputRange(ShooterConstants.kHoodWheelMinOutput, ShooterConstants.kHoodWheelMaxOutput);

    hoodWheelPIDController.setSmartMotionMaxVelocity(ShooterConstants.kHoodWheelmaxRPM, smartMotionSlot);
    hoodWheelPIDController.setSmartMotionMinOutputVelocity(50, smartMotionSlot);
    hoodWheelPIDController.setSmartMotionMaxAccel(3000, smartMotionSlot);
    hoodWheelPIDController.setSmartMotionAllowedClosedLoopError(0, smartMotionSlot);
    hoodWheelPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, smartMotionSlot);

    // Hood Wheel motor configuration
    turretMotor = new CANSparkMax(ShooterConstants.kTurretMotorPort, MotorType.kBrushless);
    turretMotor.restoreFactoryDefaults();
    turretMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    turretPIDController = turretMotor.getPIDController();
    turretEncoder = turretMotor.getEncoder();

    // set Hood Wheel PID coefficients
    turretPIDController.setP(ShooterConstants.kTurretP);
    turretPIDController.setI(ShooterConstants.kTurretI);
    turretPIDController.setD(ShooterConstants.kTurretD);
    turretPIDController.setIZone(ShooterConstants.kTurretIz);
    turretPIDController.setFF(ShooterConstants.kTurretFF);
    turretPIDController.setOutputRange(ShooterConstants.kTurretMinOutput, ShooterConstants.kTurretMaxOutput);

    turretPIDController.setSmartMotionMaxVelocity(ShooterConstants.kTurretmaxRPM, smartMotionSlot);
    turretPIDController.setSmartMotionMinOutputVelocity(50, smartMotionSlot);
    turretPIDController.setSmartMotionMaxAccel(3000, smartMotionSlot);
    turretPIDController.setSmartMotionAllowedClosedLoopError(0, smartMotionSlot);
    turretPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, smartMotionSlot);

  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber("Shooter RPM", getShooterSpeed());
    SmartDashboard.putNumber("HoodWheel RPM", getHoodWheelSpeed());
    SmartDashboard.putBoolean("Shooter at Speed", shooterIsReady());
    
  }

  public void enableShooter(){
    shooterPIDController.setReference(shooterRPMOpPoint, VelocityControlMode);
    hoodWheelPIDController.setReference(hoodRPMOpPoint, VelocityControlMode);


    lastShooterSetRPM = shooterRPMOpPoint;
    lastHoodWheelSetRPM = hoodRPMOpPoint;
    shooterIsRunning = true;
    //System.out.println("Shooter Setpoint =" + shooterRPMOpPoint);
  }

  public void enableShooter(double shooterRPM, double hoodWheelRPM) {
    shooterPIDController.setReference(shooterRPM, VelocityControlMode);
    hoodWheelPIDController.setReference(hoodWheelRPM, VelocityControlMode);

    lastShooterSetRPM = shooterRPM;
    lastHoodWheelSetRPM = hoodWheelRPM;
    shooterIsRunning = true;
  }

  public void disableShooter(){
    shooterMotor.stopMotor();
    hoodWheelMotor.stopMotor();

    lastShooterSetRPM = 0;
    lastHoodWheelSetRPM = 0;
    shooterIsRunning = false;
  }

  public boolean shooterIsReady(){
    double errorLimit = 0.03;
    
    double currentShooter = getShooterSpeed();
    boolean shooterOK = (Math.abs(lastShooterSetRPM - currentShooter)) / lastShooterSetRPM < errorLimit;

    double currentHoodWheel = getHoodWheelSpeed();
    boolean hoodWheelOK = (Math.abs(lastHoodWheelSetRPM - currentHoodWheel)) / lastHoodWheelSetRPM < errorLimit;
    
    return (shooterOK && hoodWheelOK);
  }

  public double getShooterSpeed(){
    return shooterEncoder.getVelocity();
  }

  public double getHoodWheelSpeed() {
    return hoodWheelEncoder.getVelocity();
  }

  public void setHoodPosition(double percent) {
    hoodPIDController.setReference(percent * ShooterConstants.kHoodMaxCounts, CANSparkMax.ControlType.kSmartMotion);
  }

  public double holdHooodPosition(){
    double position = getHoodPosition();
    hoodPIDController.setReference(getHoodPosition(), CANSparkMax.ControlType.kSmartMotion);
    return position;
  }

  public double getHoodPosition(){
    return hoodEncoder.getPosition();
  }

  public void setHoodPower(double power){
    hoodMotor.set(power);
  }


  public void setTurretPosition(double percent) {
    if (percent < 0){
      // kTurretMinCounts is assumed to be a negative number
      turretPIDController.setReference(-percent * ShooterConstants.kTurretMinCounts, CANSparkMax.ControlType.kSmartMotion);
    } else {
      turretPIDController.setReference(percent * ShooterConstants.kTurretMaxCounts, CANSparkMax.ControlType.kSmartMotion);
    }
  }

  public void setTurretPower(double power){
    turretMotor.set(power);
  }

  public void shoot(){
    shooterFeederMotor.set(1);
    m_timer.reset();
    m_timer.start();
  }

  public boolean shotIsDone(){
    return (m_timer.get() > ShooterConstants.kSHOT_TIME);
  }

  public void holdShot(){
    shooterFeederMotor.set(ShooterConstants.kFeederHoldPower);
    //shooterFeederMotor.stopMotor();
  }

  public void getTimer(){
    System.out.println("Timer = " + m_timer.get());
  }

  public double getlastShooterSetRPM(){
    return lastShooterSetRPM;
  }

  public double getShooterRPMOpPoint() {
    return shooterRPMOpPoint;
  }

  public void setShooterRPMOpPoint(double shooterRPMOpPoint) {
    this.shooterRPMOpPoint = shooterRPMOpPoint;
  }

  public double getHoodRPMOpPoint() {
    return hoodRPMOpPoint;
  }

  public void setHoodRPMOpPoint(double hoodRPMOpPoint) {
    this.hoodRPMOpPoint = hoodRPMOpPoint;
  }

  public void changeShooterRPM(boolean setHigh) {
    if(setHigh){
      shooterRPMOpPoint = ShooterConstants.kShootHighRPM;
      hoodRPMOpPoint = ShooterConstants.kHoodWheelHighRPM;
    } else {
      shooterRPMOpPoint = ShooterConstants.kShootLowRPM;
      hoodRPMOpPoint = ShooterConstants.kHoodWheelLowRPM;
    }

    if(shooterIsRunning){
      enableShooter();
    }
  }

  public void setShooterRPMHigh(){
    changeShooterRPM(true);
  }

  public void setShooterRPMLow(){
    changeShooterRPM(false);
  }

}
