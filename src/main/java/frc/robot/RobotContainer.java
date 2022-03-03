// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.util.List;
import java.util.Map;

import static java.util.Map.entry;

import frc.robot.commands.*;
import frc.robot.customClass.TriggerButton;
import frc.robot.subsystems.*;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {  
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private boolean fieldRelative = true;

  private final IntakePackage m_intakePackage = new IntakePackage();
  private final Intake m_intake = new Intake();
  private final Climber m_climber = new Climber();
  private final Intestine m_intestine = new Intestine();
  private final Shooter m_shooter = new Shooter();
  

  private final StickAssignmentState rightStickIsClimber = new StickAssignmentState(false);

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_manipulatorController = new XboxController(OIConstants.kManipulatorControllerPort);

  //final JoystickButton unpackButton = new JoystickButton(m_manipulatorController, XboxController.Button.kRightBumper.value);
  //final JoystickButton packButton = new JoystickButton(m_manipulatorController, XboxController.Button.kLeftBumper.value);
  //final JoystickButton stopPackButton = new JoystickButton(m_manipulatorController, XboxController.Button.kA.value);

  final JoystickButton intakeInButton = new JoystickButton(m_manipulatorController, XboxController.Button.kX.value);
  final JoystickButton intakeOutButton = new JoystickButton(m_manipulatorController, XboxController.Button.kB.value);
  // Intesting now controlled along with intake
  //final JoystickButton intestineInButton = new JoystickButton(m_manipulatorController, XboxController.Button.kY.value);
  //final JoystickButton intestineOutButton = new JoystickButton(m_manipulatorController, XboxController.Button.kA.value);
  final JoystickButton stopIntakeandIntestineButton = new JoystickButton(m_manipulatorController, XboxController.Button.kA.value);

  final POVButton setShooterHighButton = new POVButton(m_manipulatorController, 0);
  final POVButton setShooterLowButton = new POVButton(m_manipulatorController, 180);

  final TriggerButton shootTrigger = new TriggerButton(m_manipulatorController, XboxController.Axis.kRightTrigger);
  final JoystickButton enableShooterButton = new JoystickButton(m_manipulatorController, XboxController.Button.kRightBumper.value);
  final JoystickButton disableShooterButton = new JoystickButton(m_manipulatorController, XboxController.Button.kLeftBumper.value);

  final JoystickButton rightStickModeButton = new JoystickButton(m_manipulatorController, XboxController.Button.kStart.value);

  private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  PIDController customAnglePID = new PIDController(0.6, 0, 0);

  private enum CommandsToChoose {
    Default, Simple;
  }

  //private final Command DefaultAuto;
  //private final Command Simple;
  
  private final SendableChooser<CommandsToChoose> m_chooser = new SendableChooser<>();
  private final Command m_selectCommand = null;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    SmartDashboard.putString("Robot Type", Constants.robotType.toString());
    
    // Configure the button bindings
    configureButtonBindings();

    SendableRegistry.setName(m_shooter, "Shooter");
    SendableRegistry.setName(m_intake, "Intake");
    SendableRegistry.setName(m_intestine, "Intestine");

    // Setup auto command chooser
    // m_selectCommand = new SelectCommand(Map.ofEntries(
    //   entry(CommandsToChoose.Default, DefaultAuto),
    //   entry(CommandsToChoose.Simple, DefaultAuto)
    //   ), this::select);

    m_chooser.setDefaultOption("Default Auto", CommandsToChoose.Default);
    m_chooser.addOption("Simple", CommandsToChoose.Simple);
    
    Shuffleboard.getTab("Auto").add(m_chooser);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    customAnglePID.enableContinuousInput(-Math.PI, Math.PI);
    Runnable Control = () -> {

      SmartDashboard.putNumber("LeftY", m_driverController.getLeftY());
      SmartDashboard.putNumber("LeftX", m_driverController.getLeftX());
      SmartDashboard.putNumber("RightX", m_driverController.getRightX());

      // Swerve xSpeed is the vertical/forward (negative because stick is inverse)
      double xSpeed = DriveConstants.drivePercentScale * DriveConstants.kMaxSpeedMetersPerSecond
          * xSpeedLimiter.calculate(new_deadzone(-m_driverController.getLeftY()));

      // Swerve ySpeed is the sideways left/right movement (negative because chassis
      // +y is LEFT)
      double ySpeed = DriveConstants.drivePercentScale * DriveConstants.kMaxSpeedMetersPerSecond
          * ySpeedLimiter.calculate(new_deadzone(-m_driverController.getLeftX()));

      // Swerve rotation is the counter-clockwise rotation of the robot (negate stick
      // input)
      double rotation = DriveConstants.drivePercentScale * DriveConstants.kMaxSpeedMetersPerSecond
          * DriveConstants.rotRateModifier * rotLimiter.calculate(new_deadzone(-m_driverController.getRightX()));

      // Swap field/robot relative mode
      if (m_driverController.getLeftBumper()) {
        fieldRelative = true;
      } else if (m_driverController.getRightBumper()) {
        fieldRelative = false;
      }

      // Reset Gyro
      if (m_driverController.getPOV() == 0){
        m_robotDrive.zeroHeading();
      }

      // Call the Method
      SmartDashboard.putNumber("xSpeed", xSpeed);
      SmartDashboard.putNumber("ySpeed", ySpeed);
      SmartDashboard.putNumber("rotation", rotation);
      SmartDashboard.putBoolean("Field Rel", fieldRelative);
      m_robotDrive.drive(xSpeed, ySpeed, rotation, fieldRelative);

      //Check for manipulator right stick mode change

    };
    m_robotDrive.setDefaultCommand(new RunCommand(Control, m_robotDrive));

    Runnable ControlIntakePackage = () -> {
      // Intake Packge control by left manipulator stick
      double packPower = new_deadzone(m_manipulatorController.getLeftY())/4;      

      m_intakePackage.packControl(packPower);
    };
    m_intakePackage.setDefaultCommand(new RunCommand(ControlIntakePackage, m_intakePackage));

    // **** Moved to a separte declaration
    // TODO: Delete if code works
    // Runnable ControlClimber = () -> {
    //   // Climber control by right manipulator stick
    //   double liftPower = - new_deadzone(m_manipulatorController.getRightY())/2;
    //   double armPower = new_deadzone(m_manipulatorController.getRightX())/4;      
  
    //   m_climber.runLift(liftPower);
    //   m_climber.runArm(0.2 * armPower);
    // };
  
    m_climber.setDefaultCommand(new RunCommand(ControlClimber, m_climber));
    m_shooter.setDefaultCommand(new RunCommand(ControlTurret, m_shooter));

    //packButton.whenPressed(new Pack(m_intakePackage));
    //unpackButton.whenPressed(new UnPack(m_intakePackage));

    //stopPackButton.whenPressed(new InstantCommand(m_intakePackage::packOff, m_intakePackage));

    intakeInButton.whenPressed(new InstantCommand(m_intake::intakeIn, m_intake)
    .andThen(new InstantCommand(m_intestine::intestineIn, m_intestine)));
    //.whenReleased(new InstantCommand(m_intake::intakeOff, m_intake));

    intakeOutButton.whenPressed(new InstantCommand(m_intake::intakeOut, m_intake)
    .andThen(new InstantCommand(m_intestine::intestineOut, m_intestine)));
    //.whenReleased(new InstantCommand(m_intake::intakeOff, m_intake));

    //intestineInButton.whenPressed(new InstantCommand(m_intestine::intestineIn, m_intestine));
    //intestineOutButton.whenPressed(new InstantCommand(m_intestine::intestineOut, m_intestine));

    stopIntakeandIntestineButton.whenPressed(new InstantCommand(m_intestine::intestineOff, m_intestine).
    andThen(new InstantCommand(m_intake::intakeOff, m_intake)));

    shootTrigger.whenPressed(new Shoot(m_shooter));
    enableShooterButton.whenPressed(new InstantCommand(m_shooter::enableShooter,m_shooter));
    disableShooterButton.whenPressed(new InstantCommand(m_shooter::disableShooter,m_shooter));

    rightStickModeButton.toggleWhenPressed(new StartEndCommand(rightStickIsClimber::toggle,rightStickIsClimber::toggle));
  }

  double new_deadzone(double x) {
    if (Math.abs(x) > 0.08) {
      return x;
    } else {
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
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     config);

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          new Pose2d(0, 0, new Rotation2d(0)),
          // Drive Forward
          List.of(new Translation2d(1.0, 1)),
          // End 3 meters straight ahead of where we started, facing forward
          new Pose2d(2, 2, new Rotation2d(Math.toRadians(90))),
          config);

    var thetaController = new ProfiledPIDController(
        2, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(2, 0, 0),
        new PIDController(2, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }

  //*******************************************************
  // The following Commands and Runnable allow swapping 
  // right stick to control climber or shooter.
  // Using commands to make easy use of the .whenToggled 
  // Button method.
  //*******************************************************

  private Runnable ControlClimber = () -> {
    if (rightStickIsClimber.get()) {
      // Climber control by right manipulator stick
      double liftPower = -new_deadzone(m_manipulatorController.getRightY()) / 2;
      double armPower = new_deadzone(m_manipulatorController.getRightX()) / 4;

      m_climber.runLift(liftPower);
      m_climber.runArm(0.2 * armPower);
    } else {
      m_climber.runArm(0);
      m_climber.runLift(0);
    }
    SmartDashboard.putBoolean("Right Stick Climb", rightStickIsClimber.get());
  };

  private Runnable ControlTurret = () -> {
    if (!rightStickIsClimber.get()) {
      // Climber control by right manipulator stick
      double hoodPower = -new_deadzone(m_manipulatorController.getRightY()) / 4;
      double turretPower = new_deadzone(m_manipulatorController.getRightX()) / 1;

      m_shooter.setHoodPower(hoodPower);
      m_shooter.setTurretPower(turretPower);

      double prevShooterRPM = m_shooter.getShooterRPMOpPoint();

      if(setShooterHighButton.get()){
        m_shooter.setShooterRPMOpPoint(ShooterConstants.kShootHighRPM);
      } else if (setShooterLowButton.get()){
        m_shooter.setShooterRPMOpPoint(ShooterConstants.kShootLowRPM);
      }

      if((m_shooter.getShooterRPMOpPoint() != prevShooterRPM) && (m_shooter.getlastShooterSetRPM() != 0)){
        m_shooter.enableShooter();
      }
      
    } else {
      //Probably need to modify this to hold last position using PID, especially the hood position 
      m_shooter.setHoodPower(0);
      m_shooter.setTurretPower(0);
    }
  };

}