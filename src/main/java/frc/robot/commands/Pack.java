// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakePackage;

public class Pack extends CommandBase {
  private final IntakePackage m_intake;

  /** Creates a new Pack. */
  public Pack(IntakePackage intake) {
    m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.pack();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.packOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intake.packLimitHit();
  }
}
