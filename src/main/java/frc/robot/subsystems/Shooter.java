// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
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
  ControlType VelocityControlMode = ControlType.kSmartVelocity;
  private Timer m_timer = new Timer();

  
  private CANSparkMax shooterFeederMotor;
  private CANSparkMax shooterMotor;
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
    shooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    shooterPIDController = shooterMotor.getPIDController();
    shooterEncoder = shooterMotor.getEncoder();

    // set Shooter PID coefficients
    shooterPIDController.setP(ShooterConstants.kShooterP);
    shooterPIDController.setI(ShooterConstants.kShooterI);
    shooterPIDController.setD(ShooterConstants.kShooterD);
    shooterPIDController.setIZone(ShooterConstants.kShooterIz);
    shooterPIDController.setFF(ShooterConstants.kShooterFF);
    shooterPIDController.setOutputRange(ShooterConstants.kShooterMaxOutput, ShooterConstants.kShooterMinOutput);
    
    shooterPIDController.setSmartMotionMaxVelocity(ShooterConstants.kShootermaxRPM, smartMotionSlot);
    shooterPIDController.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
    shooterPIDController.setSmartMotionMaxAccel(3000, smartMotionSlot);
    shooterPIDController.setSmartMotionAllowedClosedLoopError(0, smartMotionSlot);
    shooterPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, smartMotionSlot);

    // Hood motor configuration
    hoodMotor = new CANSparkMax(ShooterConstants.kHoodMotorPort, MotorType.kBrushless);
    hoodMotor.restoreFactoryDefaults();
    hoodMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    hoodPIDController = hoodMotor.getPIDController();
    hoodEncoder = hoodMotor.getEncoder();

    // set Hood PID coefficients
    hoodPIDController.setP(ShooterConstants.kHoodP);
    hoodPIDController.setI(ShooterConstants.kHoodI);
    hoodPIDController.setD(ShooterConstants.kHoodD);
    hoodPIDController.setIZone(ShooterConstants.kHoodIz);
    hoodPIDController.setFF(ShooterConstants.kHoodFF);
    hoodPIDController.setOutputRange(ShooterConstants.kHoodMaxOutput, ShooterConstants.kHoodMinOutput);

    hoodPIDController.setSmartMotionMaxVelocity(ShooterConstants.kHoodmaxRPM, smartMotionSlot);
    hoodPIDController.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
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
    hoodWheelPIDController.setOutputRange(ShooterConstants.kHoodWheelMaxOutput, ShooterConstants.kHoodWheelMinOutput);

    hoodWheelPIDController.setSmartMotionMaxVelocity(ShooterConstants.kHoodWheelmaxRPM, smartMotionSlot);
    hoodWheelPIDController.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
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
    turretPIDController.setOutputRange(ShooterConstants.kTurretMaxOutput, ShooterConstants.kTurretMinOutput);

    turretPIDController.setSmartMotionMaxVelocity(ShooterConstants.kTurretmaxRPM, smartMotionSlot);
    turretPIDController.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
    turretPIDController.setSmartMotionMaxAccel(3000, smartMotionSlot);
    turretPIDController.setSmartMotionAllowedClosedLoopError(0, smartMotionSlot);
    turretPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, smartMotionSlot);

  }

  public void enableShooter(){
    shooterPIDController.setReference(ShooterConstants.kShooterTypRPM, VelocityControlMode);
    hoodWheelPIDController.setReference(ShooterConstants.kHoodWheelTypRPM, VelocityControlMode);

    lastShooterSetRPM = ShooterConstants.kShooterTypRPM;
    lastHoodWheelSetRPM = ShooterConstants.kHoodWheelTypRPM;
  }

  public void enableShooter(double shooterRPM, double hoodWheelRPM) {
    shooterPIDController.setReference(shooterRPM, VelocityControlMode);
    hoodWheelPIDController.setReference(hoodWheelRPM, VelocityControlMode);

    lastShooterSetRPM = shooterRPM;
    lastHoodWheelSetRPM = hoodWheelRPM;
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
    shooterFeederMotor.set(IntestineConstants.kIntestinePower);
    m_timer.reset();
  }

  public boolean shotIsDone(){
    return (m_timer.hasElapsed(ShooterConstants.kSHOT_TIME));
  }

  public void holdShot(){
    shooterFeederMotor.set(ShooterConstants.kFeederHoldPower);
  }

}
