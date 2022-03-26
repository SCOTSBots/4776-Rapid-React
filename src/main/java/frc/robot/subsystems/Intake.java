// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotor;
  private boolean isRunning = false;

  /** Creates a new Intake. */
  public Intake() {
    //Assume postive motor power pulls balls in
    intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorPort, MotorType.kBrushless);
    intakeMotor.setInverted(true);
  }

  public void runIntake(double power){
    intakeMotor.set(power);
    isRunning = true;
  }
  public void intakeIn(){
    intakeMotor.set(IntakeConstants.intakePower);
    isRunning = true;
  }
  public void intakeOut(){
    intakeMotor.set(-IntakeConstants.intakePower);
    isRunning = true;
  }
  public void intakeOff(){
    intakeMotor.stopMotor();
    isRunning = false;
  }

  public boolean getIsRunning(){
    return isRunning;
  }
}
