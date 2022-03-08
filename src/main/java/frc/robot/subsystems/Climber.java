// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private final CANSparkMax liftMotor;
  private final CANSparkMax armMotor;
  private final CANSparkMax armMotor2;

  /** Creates a new Intake. */
  public Climber() {
    //Configure so that postive motor power raises robot
    liftMotor = new CANSparkMax(ClimberConstants.liftMotorPort, MotorType.kBrushless);
    
    //Configure so that positive motor power moves arm towards front of robot
    armMotor = new CANSparkMax(ClimberConstants.armMotorPort, MotorType.kBrushless);
    armMotor.setInverted(true);
    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor2 = new CANSparkMax(ClimberConstants.armMotor2Port, MotorType.kBrushless);
    armMotor.setInverted(false);
    armMotor2.setIdleMode(IdleMode.kBrake);

  }



  public void runLift(double power){
    liftMotor.set(power);
  }
  public void runArm(double power){
    armMotor.set(power);
    armMotor2.set(power);
  }

}
