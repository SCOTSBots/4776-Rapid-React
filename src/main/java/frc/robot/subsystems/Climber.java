// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private final CANSparkMax liftMotorLeft;
  private final CANSparkMax liftMotorRight;
  private final CANSparkMax feederMotorLeft;
  private final CANSparkMax feederMotorRight;
  private final CANSparkMax armMotors;

  /** Creates a new Intake. */
  public Climber() {
    //Configure so that postive motor power raises robot
    liftMotorLeft = new CANSparkMax(ClimberConstants.liftMotorLeftPort, MotorType.kBrushless);
    liftMotorLeft.setIdleMode(IdleMode.kBrake);
    liftMotorLeft.setInverted(false);
    liftMotorRight = new CANSparkMax(ClimberConstants.liftMotorRightPort, MotorType.kBrushless);
    liftMotorRight.setIdleMode(IdleMode.kBrake);
    liftMotorRight.setInverted(true);
    //liftMotor.setSmartCurrentLimit(40);
    
    feederMotorLeft = new CANSparkMax(ClimberConstants.feederMotorLeftPort, MotorType.kBrushless);
    feederMotorLeft.setInverted(true);
    feederMotorLeft.setIdleMode(IdleMode.kBrake);
    feederMotorRight = new CANSparkMax(ClimberConstants.feederMotorRightPort, MotorType.kBrushless);
    feederMotorRight.setInverted(false);
    feederMotorRight.setIdleMode(IdleMode.kBrake);

    armMotors = null;
    //armMotors = new CANSparkMax(ClimberConstants.armMotorsPort, MotorType.kBrushed);
    //armMotors.setIdleMode(IdleMode.kBrake);
    //armMotors.setSmartCurrentLimit(20);

  }

  public void runLift(double power) {
    runRightLift(power);
    runLeftLift(power);
  }

  public void runLeftLift(double power) {
    runLiftSide(liftMotorLeft, feederMotorLeft, power);
  }

  public void runRightLift(double power) {
    runLiftSide(liftMotorRight, feederMotorRight, power);
  }

  public void runLiftSide(CANSparkMax liftMotor, CANSparkMax feederMotor, double power) {
    if (power > 0) {
      liftMotor.setIdleMode(IdleMode.kCoast);
      feederMotor.setIdleMode(IdleMode.kBrake);
      feederMotor.set(power/3);
      liftMotor.set(0);
    } else  {
      liftMotor.setIdleMode(IdleMode.kBrake);
      feederMotor.setIdleMode(IdleMode.kCoast);
      liftMotor.set(power);
      feederMotor.set(0);
    }
  }

  public void runArm(double power) {
    //armMotors.set(power);
  }

  public double getClimberCurrent(){
    return liftMotorLeft.getOutputCurrent();
  }

}
