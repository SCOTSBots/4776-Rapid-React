// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WallGrabShootShoot extends SequentialCommandGroup {
  public WallGrabShootShoot(DriveSubsystem drive, Shooter shooter, IntakePackage intakePackage, Intake intake,
      Intestine intestine, Climber climber) {
    /** Creates a new GrabShootShoot. */

    final double DISTANCE = 1.25;

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    Trajectory driveToBallTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Drive Forward
        List.of(new Translation2d(DISTANCE*0.8, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(DISTANCE, -0.7, new Rotation2d(Math.toRadians(-90))),
        config);


    Trajectory driveToShootTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(DISTANCE, -0.7, new Rotation2d(-90)),
        // Drive Forward
        List.of(new Translation2d(DISTANCE/2, -1.25)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(0, -0.75, new Rotation2d(Math.toRadians(-200.0))),
        config);

    var thetaController = new ProfiledPIDController(
        2, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand driveToBall = new SwerveControllerCommand(
        driveToBallTrajectory,
        drive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(2, 0, 0),
        new PIDController(2, 0, 0),
        thetaController,
        drive::setModuleStates,
        drive);

        SwerveControllerCommand driveToShoot = new SwerveControllerCommand(
        driveToShootTrajectory,
        drive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(2, 0, 0),
        new PIDController(2, 0, 0),
        thetaController,
        drive::setModuleStates,
        drive);


    // Reset odometry to the starting pose of the trajectory.
    drive.resetOdometry(driveToBallTrajectory.getInitialPose());

    addCommands(
        // Confiigure and spinup shooter
        new InstantCommand(() -> {
          shooter.setShooterConfig(Constants.ShooterConstants.shootAutoHighClose);
          shooter.enableShooter();
        }, shooter),

        // Unpack and start the intake
        new ParallelCommandGroup(
            new UnPack(intakePackage),
            new InstantCommand(intake::intakeIn, intake),
            new InstantCommand(intestine::intestineIn, intestine)),

        // Drive to the ball and turn to shoot
        driveToBall.andThen(() -> drive.drive(0, 0, 0, false)),
        new WaitCommand(0.5),
        driveToShoot.andThen(() -> drive.drive(0, 0, 0, false)),

        // Shoot shoot
        new Shoot(shooter),
        new WaitCommand(1),
        new Shoot(shooter),

        // Shut it down
        new InstantCommand(shooter::disableShooter, shooter),
        new InstantCommand(intestine::intestineOff, intestine),
        new InstantCommand(intake::intakeOff, intake),
        new InstantCommand(shooter::stopHood, shooter));
  }

}
