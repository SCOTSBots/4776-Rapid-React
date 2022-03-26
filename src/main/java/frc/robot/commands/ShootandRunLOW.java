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

public class ShootandRunLOW extends SequentialCommandGroup {
        /**
         * Creates a new ComplexAuto.
         *
         * 
         */
        public ShootandRunLOW(DriveSubsystem drive, Shooter shooter, IntakePackage intakePackage, Intake intake,
                        Intestine intestine, Climber climber) {
                // Create config for trajectory
                TrajectoryConfig config = new TrajectoryConfig(
                                AutoConstants.kMaxSpeedMetersPerSecond,
                                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                                // Add kinematics to ensure max speed is actually obeyed
                                                .setKinematics(DriveConstants.kDriveKinematics).setReversed(true);

                Trajectory driveToBallTrajectory = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                new Pose2d(0, 0, new Rotation2d(0)),
                                // Drive Forward
                                List.of(new Translation2d(-1.0, 0)),
                                // End 3 meters straight ahead of where we started, facing forward
                                new Pose2d(-2.0, 0, new Rotation2d(Math.toRadians(0))),
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

                // Reset odometry to the starting pose of the trajectory.
                drive.resetOdometry(driveToBallTrajectory.getInitialPose());

                addCommands(
                                // Confiigure and spinup shooter and move arm
                                new InstantCommand(() -> {
                                        shooter.setShooterConfig(Constants.ShooterConstants.shootAutoClose);
                                        shooter.enableShooter();
                                        // climber.runArm(0.5);
                                }, shooter),
                                // new WaitCommand(0.5),
                                // new InstantCommand(()->{
                                // climber.runArm(0);
                                // }, climber),

                                // Unpack and start the intake
                                new ParallelCommandGroup(
                                                new UnPack(intakePackage),
                                                new InstantCommand(intestine::intestineIn, intestine)),

                                // Shoot
                                new WaitCommand(2.5),
                                new Shoot(shooter),

                                // Shut it down
                                new InstantCommand(shooter::disableShooter, shooter),
                                new InstantCommand(intestine::intestineOff, intestine),
                                new InstantCommand(intake::intakeOff, intake),

                                // Drive to the ball
                                driveToBall.andThen(() -> drive.drive(0, 0, 0, false)));
        }
}
