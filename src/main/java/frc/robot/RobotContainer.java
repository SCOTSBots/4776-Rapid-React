// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  //XboxController m_manipulatorController = new XboxController(OIConstants.kManipulatorControllerPort);

  private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  PIDController customAnglePID = new PIDController(0.6, 0, 0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
  //   m_robotDrive.setDefaultCommand(
  //       // A split-stick arcade command, with forward/backward controlled by the left
  //       // hand, and turning controlled by the right.
  //       new RunCommand(
  //           () ->
  //               m_robotDrive.drive(
  //                   m_driverController.getLeftY(),
  //                 m_driverController.getRightX(),
  //                   m_driverController.getLeftX(),
  //                   false))
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
      customAnglePID.enableContinuousInput(-Math.PI, Math.PI);
      Runnable Control = ()->{
        // m_driverController.getLeftY();
        // m_driverController.getLeftX();
        // m_driverController.getRightX();

        //Create variables for controls

        SmartDashboard.putNumber("LeftY", m_driverController.getLeftY());
        SmartDashboard.putNumber("LeftX", m_driverController.getLeftX());
        SmartDashboard.putNumber("RightX", m_driverController.getRightX());

        //Swerve xSpeed is the vertical/forward movement
        double xSpeed = DriveConstants.kMaxSpeedMetersPerSecond * xSpeedLimiter.calculate(new_deadzone(m_driverController.getLeftY()));
        
        //Swerve ySpeed is the sideways left/right movement
        double ySpeed = DriveConstants.kMaxSpeedMetersPerSecond * ySpeedLimiter.calculate(new_deadzone(m_driverController.getLeftX()));
        
        //Swerve rotation is the counter-clockwise rotation of the robot
        double rotation = DriveConstants.kMaxSpeedMetersPerSecond * rotLimiter.calculate(new_deadzone(m_driverController.getRightX()));

        boolean resetRotation = m_driverController.getRightBumper();
        // System.out.println(m_robotDrive.getPose().getRotation().getRadians());
        if (resetRotation) {
          //We want full speed rotation when angle = 45 degrees = pi/4
          rotation = m_robotDrive.getPose().getRotation().getRadians() * 8 / Math.PI;
          rotation = Math.min(1, Math.max(-1, rotation));
        }
        //TODO: Driver can OVERRIDE manipulator rotation using right stick down button
        //DIsabled Manipulator override
        double rX = 0; //m_manipulatorController.getLeftX();
        double rY = 0; //-m_manipulatorController.getLeftY();
        if (rX * rX + rY * rY > 0.5) {
          double angle = Math.atan2(rY, rX) + Math.PI/2;
          // System.out.println("A: "+angle);
          rotation = customAnglePID.calculate(m_robotDrive.getPose().getRotation().getRadians(), angle);
        }
        //Call the Method
        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("rotation", rotation);
        m_robotDrive.drive(xSpeed, ySpeed, rotation, false);
      };
      m_robotDrive.setDefaultCommand(new RunCommand(Control,m_robotDrive));
  }
double new_deadzone(double x) {
  if (Math.abs(x) > 0.1) {
return x;
  }
  else {
    return 0;
  }
}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
  }