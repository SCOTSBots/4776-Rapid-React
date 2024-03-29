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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootNothing;
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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.util.List;
import java.util.Map;

import static java.util.Map.entry;

//import frc.robot.commands.*;
import frc.robot.commands.ShootandRunLOW;
import frc.robot.commands.ShootandRunHIGH;
import frc.robot.commands.GrabShootShoot;
import frc.robot.commands.WallGrabShootShoot;
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

  // Init Limelight
  // NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  // NetworkTableEntry txShuffle = table.getEntry("tx");
  // NetworkTableEntry tyShuffle = table.getEntry("ty");
  // NetworkTableEntry taShuffle = table.getEntry("ta");

  // Read the Limelight values periodically
  // double x = txShuffle.getDouble(0.0);
  // double y = tyShuffle.getDouble(0.0);
  // double area = taShuffle.getDouble(0.0);

  /** Creates a new LimelightSubsystem. */
  // public void LimelightSubsystem() {
  // setupLimelightShuffleBoard();
  // }

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_manipulatorController = new XboxController(OIConstants.kManipulatorControllerPort);

  POVButton resetGyro = new POVButton(m_driverController, 0);

  // final JoystickButton unpackButton = new
  // JoystickButton(m_manipulatorController,
  // XboxController.Button.kRightBumper.value);
  // final JoystickButton packButton = new JoystickButton(m_manipulatorController,
  // XboxController.Button.kLeftBumper.value);
  // final JoystickButton stopPackButton = new
  // JoystickButton(m_manipulatorController, XboxController.Button.kA.value);

  final JoystickButton intakeInButton = new JoystickButton(m_manipulatorController, XboxController.Button.kX.value);
  final JoystickButton intakeOutButton = new JoystickButton(m_manipulatorController, XboxController.Button.kB.value);
  // Intesting now controlled along with intake
  // final JoystickButton intestineInButton = new
  // JoystickButton(m_manipulatorController, XboxController.Button.kY.value);
  // final JoystickButton intestineOutButton = new
  // JoystickButton(m_manipulatorController, XboxController.Button.kA.value);
  final JoystickButton stopIntakeandIntestineButton = new JoystickButton(m_manipulatorController,
      XboxController.Button.kA.value);

  //final POVButton setShooterHighButton = new POVButton(m_manipulatorController, 0);
  final POVButton setShooterHighFar = new POVButton(m_manipulatorController, 0);
  final POVButton setShooterHighClose = new POVButton(m_manipulatorController, 180);
  //final POVButton setShooterLongshot = new POVButton(m_manipulatorController, 90);
  final JoystickButton setShooterLowGoal = new JoystickButton(m_manipulatorController, XboxController.Button.kY.value);

  final POVButton incShooterPowerButton = new POVButton(m_manipulatorController, 90);
  final POVButton decShooterPowerButton = new POVButton(m_manipulatorController, 270);

  final TriggerButton resetHood = new TriggerButton(m_manipulatorController, XboxController.Axis.kLeftTrigger);

  final TriggerButton shootTrigger = new TriggerButton(m_manipulatorController, XboxController.Axis.kRightTrigger);
  final JoystickButton enableShooterButton = new JoystickButton(m_manipulatorController,
      XboxController.Button.kRightBumper.value);
  final JoystickButton disableShooterButton = new JoystickButton(m_manipulatorController,
      XboxController.Button.kLeftBumper.value);

  final JoystickButton rightStickModeButton = new JoystickButton(m_manipulatorController,
      XboxController.Button.kStart.value);

  final TriggerButton lowSpeedTrigger = new TriggerButton(m_driverController, XboxController.Axis.kRightTrigger);

  final JoystickButton brakeButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);

  // final TriggerButton autoAimTrigger = new TriggerButton(m_driverController,
  // XboxController.Axis.kLeftTrigger);

  private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(2);

  final JoystickButton testCommandButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);

  private boolean hoodHold = false;

  PIDController customAnglePID = new PIDController(0.6, 0, 0);

  private enum CommandsToChoose {
    ShootandRunLOW, ShootandRunHIGH, GrabShootShoot, WallGrabShootShoot
  }

  public Command shootAndRunLOW;
  public Command shootAndRunHIGH;
  public Command grabShootShoot;
  public Command wallGrabShootShoot;

  private final SendableChooser<CommandsToChoose> m_chooser = new SendableChooser<>();
  private Command m_selectCommand = null;

  
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

    // Generate Auto Command Sequences
    generateAutoRoutines();

    // Setup auto command chooser
    m_selectCommand = new SelectCommand(Map.ofEntries(
        entry(CommandsToChoose.ShootandRunLOW, shootAndRunLOW),
        entry(CommandsToChoose.ShootandRunHIGH, shootAndRunHIGH),
        entry(CommandsToChoose.GrabShootShoot, grabShootShoot),
        entry(CommandsToChoose.WallGrabShootShoot, wallGrabShootShoot)), m_chooser::getSelected);

    m_chooser.setDefaultOption("Shoot and Run Low", CommandsToChoose.ShootandRunLOW);
    m_chooser.addOption("Shoot and Run High", CommandsToChoose.ShootandRunHIGH);
    m_chooser.addOption("Grab Ball and Shoot Two", CommandsToChoose.GrabShootShoot);
    m_chooser.addOption("Wall Grab Ball and Shoot Two", CommandsToChoose.WallGrabShootShoot);

    Shuffleboard.getTab("Auto").add(m_chooser)
    .withPosition(0, 0)
    .withSize(7, 2);

    // // Post the Limelight values to SmartDashboard periodically
    // SmartDashboard.putNumber("LimelightX", x);
    // SmartDashboard.putNumber("LimelightY", y);
    // SmartDashboard.putNumber("LimelightArea", area);

  }

  // private void setupLimelightShuffleBoard() {
  // final ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");

  // txShuffle = limelightTab.add("LimelightX", x).getEntry();
  // tyShuffle = limelightTab.add("LimelightY", y).getEntry();
  // taShuffle = limelightTab.add("LimelightArea", area).getEntry();
  // }

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

      // SmartDashboard.putNumber("LeftY", m_driverController.getLeftY());
      // SmartDashboard.putNumber("LeftX", m_driverController.getLeftX());
      // SmartDashboard.putNumber("RightX", m_driverController.getRightX());

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
      // if (m_driverController.getPOV() == 0) {
      //   m_robotDrive.zeroHeading();
      //   System.out.println("Zeroing");
      // }

      // Call the Method
      // SmartDashboard.putNumber("xSpeed", xSpeed);
      // SmartDashboard.putNumber("ySpeed", ySpeed);
      // SmartDashboard.putNumber("rotation", rotation);
      SmartDashboard.putBoolean("Field Rel", fieldRelative);
      m_robotDrive.drive(xSpeed, ySpeed, rotation, fieldRelative);

      // System.out.println("Starting Pose Angle" +
      // m_robotDrive.getPose().getRotation().getDegrees());

    };
    m_robotDrive.setDefaultCommand(new RunCommand(Control, m_robotDrive));

    Runnable ControlIntakePackage = () -> {
      // Intake Packge control by left manipulator stick
      if (!rightStickIsClimber.get()) {
        double packPower = new_deadzone(m_manipulatorController.getLeftY());

        if (packPower < 0) {
          packPower = packPower * 0.5;
        } else {
          packPower = packPower * 0.25;
        }

        m_intakePackage.packControl(packPower);
      } else {
        m_intakePackage.packControl(0);
      }
    };
    m_intakePackage.setDefaultCommand(new RunCommand(ControlIntakePackage, m_intakePackage));

    m_climber.setDefaultCommand(new RunCommand(ControlClimber, m_climber));
    m_shooter.setDefaultCommand(new RunCommand(ControlTurret, m_shooter));

    lowSpeedTrigger.whenPressed(new InstantCommand(m_robotDrive::setSlowDrive, m_robotDrive))
        .whenReleased(new InstantCommand(m_robotDrive::setNormalDrive, m_robotDrive));
    // packButton.whenPressed(new Pack(m_intakePackage));
    // unpackButton.whenPressed(new UnPack(m_intakePackage));

    // stopPackButton.whenPressed(new InstantCommand(m_intakePackage::packOff,
    // m_intakePackage));

    intakeInButton.whenPressed(new InstantCommand(m_intake::intakeIn, m_intake)
        .andThen(new InstantCommand(m_intestine::intestineIn, m_intestine)));
    // .whenReleased(new InstantCommand(m_intake::intakeOff, m_intake));

    intakeOutButton.whenPressed(new InstantCommand(m_intake::intakeOut, m_intake)
        .andThen(new InstantCommand(m_intestine::intestineOut, m_intestine)));
    // .whenReleased(new InstantCommand(m_intake::intakeOff, m_intake));

    // intestineInButton.whenPressed(new InstantCommand(m_intestine::intestineIn,
    // m_intestine));
    // intestineOutButton.whenPressed(new InstantCommand(m_intestine::intestineOut,
    // m_intestine));

    stopIntakeandIntestineButton.whenPressed(new InstantCommand(() -> {
      m_intestine.intestineSmartOff(m_shooter.getShooterIsRunning());
    }, m_intestine)
    .andThen(new InstantCommand(m_intake::intakeOff, m_intake)));

    // Shooter controls
    shootTrigger.whenPressed(new ConditionalCommand(new ShootNothing(m_shooter), new Shoot(m_shooter), rightStickIsClimber::get));

    enableShooterButton.whenPressed(new InstantCommand(m_shooter::enableShooter, m_shooter)
    .andThen(new InstantCommand(m_intestine::intestineIn, m_intestine)));
    disableShooterButton.whenPressed(new InstantCommand(m_shooter::disableShooter, m_shooter)
    .andThen(new InstantCommand(() -> {
      m_intestine.intestineSmartOff(m_intake.getIsRunning());
    }, m_intestine)));
    
    // setShooterHighButton.whenPressed(new InstantCommand(() -> {
    //   m_shooter.setShooterConfig(ShooterConstants.shootAutoOutside);
    // }, m_shooter))
    // .whenReleased(new InstantCommand(m_shooter::stopHood, m_shooter));
    
    setShooterHighFar.whenPressed(new InstantCommand(() -> {
      m_shooter.setShooterConfig(ShooterConstants.shootAutoOutside);
    }, m_shooter))
    .whenReleased(new InstantCommand(m_shooter::stopHood, m_shooter));
    
    setShooterHighClose.whenPressed(new InstantCommand(() -> {
      m_shooter.setShooterConfig(ShooterConstants.shootAutoHighClose);
    }, m_shooter))
    .whenReleased(new InstantCommand(m_shooter::stopHood, m_shooter));

    setShooterLowGoal.whenPressed(new InstantCommand(() -> {
      m_shooter.setShooterConfig(ShooterConstants.shootAutoClose);
    }, m_shooter))
    .whenReleased(new InstantCommand(m_shooter::stopHood, m_shooter));

    incShooterPowerButton.whenPressed(m_shooter::incShooterSpeed, m_shooter);
    decShooterPowerButton.whenPressed(m_shooter::decShooterSpeed, m_shooter);

    // setShooterLongshot.whenPressed(new InstantCommand(() -> {
    //   m_shooter.setShooterConfig(ShooterConstants.shootLongShot);
    // }, m_shooter))
    // .whenReleased(new InstantCommand(m_shooter::stopHood, m_shooter));

    resetHood.whileHeld(new InstantCommand(() -> {
      if (!rightStickIsClimber.get()) {
        m_shooter.setHoodPower(-0.5);
      }
    }, m_shooter))
        .whenReleased(new InstantCommand(() -> {
          if (!rightStickIsClimber.get()) {
            m_shooter.setHoodPower(0);
            m_shooter.resetHoodEncoder();
          }
        }, m_shooter));

    // TODO: Finish Limelight code w/ help
    // autoAimTrigger.whenPressed()

    rightStickModeButton
        .toggleWhenPressed(new StartEndCommand(rightStickIsClimber::toggle, rightStickIsClimber::toggle));

    resetGyro.whenPressed(new InstantCommand(m_robotDrive::zeroHeading, m_robotDrive));

    brakeButton.whileHeld(new InstantCommand(m_robotDrive::setXModuleState, m_robotDrive));

    // testCommandButton.whenPressed(new InstantCommand(()->{
    // m_robotDrive.turnByAngle(179.9);
    // }, m_robotDrive));

    // testCommandButton.whileHeld(new InstantCommand(()->{
    //   m_robotDrive.coastModuleTurn();
    //   }, m_robotDrive));

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
    return m_selectCommand;
  }

  // *******************************************************
  // The following Commands and Runnable allow swapping
  // right stick to control climber or shooter.
  // Using commands to make easy use of the .whenToggled
  // Button method.
  // *******************************************************

  private Runnable ControlClimber = () -> {
    double climberCurrent = 0;

    if (rightStickIsClimber.get()) {
      // Climber control by right manipulator stick
      double liftPowerLeft, liftPowerRight;
      if (m_manipulatorController.getLeftTriggerAxis() < 0.1) {
        liftPowerRight = -new_deadzone(m_manipulatorController.getRightY()) / 2;
        liftPowerLeft = liftPowerRight;
      } else {
        liftPowerRight = -new_deadzone(m_manipulatorController.getRightY()) / 2;
        liftPowerLeft = -new_deadzone(m_manipulatorController.getLeftY()) / 2;
      }

      double armPower;
      if(m_manipulatorController.getRightTriggerAxis() < 0.2){
        armPower = -new_deadzone(m_manipulatorController.getRightX()) * 0.8;
      } else {
        armPower = -m_manipulatorController.getRightTriggerAxis() * 0.8;
      }

      m_climber.runLeftLift(liftPowerLeft);
      m_climber.runRightLift(liftPowerRight*1.1);
      m_climber.runArm(armPower);
      climberCurrent = m_climber.getClimberCurrent();
    } else {
      m_climber.runArm(0);
      m_climber.runLift(0);
    }
    SmartDashboard.putBoolean("Right Stick Climb", rightStickIsClimber.get());
    SmartDashboard.putNumber("Climber Current", climberCurrent);
  };

  private Runnable ControlTurret = () -> {
    if (!rightStickIsClimber.get()) {
      // Climber control by right manipulator stick
      double hoodPower = -new_deadzone(m_manipulatorController.getRightY()) * 0.15;
      //double turretPower = new_deadzone(m_manipulatorController.getRightX()) / 1.5;

      if (hoodPower != 0) {
        m_shooter.setHoodPower(hoodPower);
        hoodHold = false;
        //System.out.println("Powering hood.");
      } else if (!hoodHold) {
        double position = m_shooter.holdHooodPosition();
        hoodHold = true;
        //System.out.println("Setting hold position @ " + position);
      }

      // m_shooter.setTurretPower(turretPower);

    } else {
      // Probably need to modify this to hold last position using PID, especially the
      // hood position
      // m_shooter.holdHooodPosition(); **Should already be in hold mode?
      // m_shooter.setTurretPower(0);
    }

    SmartDashboard.putNumber("Hood Position", m_shooter.getHoodPosition() / ShooterConstants.kHoodMaxCounts * 100);
  };

  // Generate auto routines
  public void generateAutoRoutines() {
    shootAndRunLOW = new ShootandRunLOW(m_robotDrive, m_shooter, m_intakePackage, m_intake, m_intestine, m_climber);
    shootAndRunHIGH = new ShootandRunHIGH(m_robotDrive, m_shooter, m_intakePackage, m_intake, m_intestine, m_climber);
    grabShootShoot = new GrabShootShoot(m_robotDrive, m_shooter, m_intakePackage, m_intake, m_intestine, m_climber);
    wallGrabShootShoot = new WallGrabShootShoot(m_robotDrive, m_shooter, m_intakePackage, m_intake, m_intestine, m_climber);
  }

  public void zeroOdo(){
    m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
  }
  

}