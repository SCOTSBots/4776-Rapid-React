// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;

public class Intestine extends SubsystemBase {
  private final CANSparkMax intestineMotor;

  /** Creates a new Intake. */
  public Intestine() {
    //Assume postive motor power pulls balls in
    intestineMotor = new CANSparkMax(IntakeConstants.intakeMotorPort, MotorType.kBrushless);
    intestineMotor.setInverted(true);

  }

  public void runIntestine(double power){
    intestineMotor.set(power);
  }
  public void intestineIn(){
    intestineMotor.set(IntakeConstants.intakePower);
  }
  public void intestineOut(){
    intestineMotor.set(-IntakeConstants.intakePower);
  }
  public void intestineOff(){
    intestineMotor.stopMotor();
  }
}
