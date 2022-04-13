// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShootNothing extends CommandBase {
  /** Creates an empty command for the shooter. */
  private Shooter m_shooter;

  public ShootNothing(Shooter shooter) {
    this.m_shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

}
