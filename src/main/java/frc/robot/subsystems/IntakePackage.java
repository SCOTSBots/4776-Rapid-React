// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;

public class IntakePackage extends SubsystemBase {
  private final CANSparkMax intakePackageMotor;


  /** Creates a new Intake. */
  public IntakePackage() {
    //Assume postive motor power unpackages the intake
    intakePackageMotor = new CANSparkMax(IntakeConstants.intakePackageMotorPort, MotorType.kBrushless);

  }

  public void pack(){
    intakePackageMotor.set(-IntakeConstants.packagePower);
  }

  public void unpack(){
    intakePackageMotor.set(IntakeConstants.unpackagePower);
  }

  public void packControl(double power){
    intakePackageMotor.set(power);
  }

  public void packOff() {
    intakePackageMotor.stopMotor();
  }

  public boolean packLimitHit() {
    return intakePackageMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).isPressed();
  }

  public boolean unpackLimitHit() {
    return intakePackageMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).isPressed();
  }

}
