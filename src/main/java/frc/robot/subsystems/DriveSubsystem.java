// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

@SuppressWarnings("PMD.ExcessiveImports")
public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort,
      DriveConstants.kFrontLeftTurningEncoderPorts,
      DriveConstants.kFrontLeftDriveEncoderReversed,
      DriveConstants.kFrontLeftTurningEncoderReversed,
      false, false,
      ModuleConstants.kFrontLeftTurningEncoderCounts);

  private final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.kRearLeftDriveMotorPort,
      DriveConstants.kRearLeftTurningMotorPort,
      DriveConstants.kRearLeftTurningEncoderPorts,
      DriveConstants.kRearLeftDriveEncoderReversed,
      DriveConstants.kRearLeftTurningEncoderReversed,
      false, false,
      ModuleConstants.kRearLeftTurningEncoderCounts);

  private final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort,
      DriveConstants.kFrontRightTurningEncoderPorts,
      DriveConstants.kFrontRightDriveEncoderReversed,
      DriveConstants.kFrontRightTurningEncoderReversed,
      false, false,
      ModuleConstants.kFrontRightTurningEncoderCounts);

  private final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.kRearRightDriveMotorPort,
      DriveConstants.kRearRightTurningMotorPort,
      DriveConstants.kRearRightTurningEncoderPorts,
      DriveConstants.kRearRightDriveEncoderReversed,
      DriveConstants.kRearRightTurningEncoderReversed,
      false, false,
      ModuleConstants.kRearRightTurningEncoderCounts);

  private final SwerveModule[] swerveModules = {
      m_frontLeft,
      m_frontRight,
      m_rearLeft,
      m_rearRight };

  // The gyro sensor
  private final Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d());

  private NetworkTableEntry[] swerveModuleShuffleTargetAngle = new NetworkTableEntry[4];
  private NetworkTableEntry[] swerveModuleShuffleTargetSpeed = new NetworkTableEntry[4];
  private NetworkTableEntry[] swerveModuleShuffleActualAngle = new NetworkTableEntry[4];
  private NetworkTableEntry[] swerveModuleShuffleActualSpeed = new NetworkTableEntry[4];

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    setupShuffleBoard();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        // new Rotation2d(getHeading()),
        m_gyro.getRotation2d(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    // System.out.println("State: "+swerveModuleStates[0].angle.getRadians()+",
    // drive: "+xSpeed+" or "+ySpeed+" when at "+m_frontLeft.getAngleRadians());

    boolean noMovement = false && xSpeed == 0 && ySpeed == 0 && rot == 0;

    for (int i = 0; i < 4; i++) {
      swerveModuleShuffleTargetAngle[i].setDouble(swerveModuleStates[i].angle.getDegrees());
      swerveModuleShuffleTargetSpeed[i].setDouble(swerveModuleStates[i].speedMetersPerSecond
          / DriveConstants.kMaxSpeedMetersPerSecond);

      swerveModules[i].setDesiredState(swerveModuleStates[i], noMovement);

      swerveModuleShuffleActualAngle[i].setDouble(swerveModules[i].getState().angle.getDegrees());
      swerveModuleShuffleActualSpeed[i].setDouble(swerveModules[i].getState().speedMetersPerSecond);
    }

  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0], false);
    m_frontRight.setDesiredState(desiredStates[1], false);
    m_rearLeft.setDesiredState(desiredStates[2], false);
    m_rearRight.setDesiredState(desiredStates[3], false);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    // m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  private void setupShuffleBoard() {
    final ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");

    // ShuffleboardLayout swerveAngleLayout = swerveTab
    // .getLayout("Swerve Angles", BuiltInLayouts.kList)
    // .withSize(6, 10)
    // .withPosition(6, 0);

    // ShuffleboardLayout swerveSpeedLayout = swerveTab
    // .getLayout("Swerve Speed", BuiltInLayouts.kList)
    // .withSize(4, 8)
    // .withPosition(14, 0);

    Shuffleboard.getTab("Swerve").addNumber("Odometry X Position", () -> this.getPose().getX());
    Shuffleboard.getTab("Swerve").addNumber("Odometry Y Position", () -> this.getPose().getY());
    Shuffleboard.getTab("Swerve").addNumber("Odometry Rotation", () -> this.getPose().getRotation().getDegrees());


    for (int i = 0; i < 4; i++) {
      swerveModuleShuffleTargetAngle[i] = swerveTab.add("M" + i + " TarAngle", 0)
          .withSize(3, 2)
          .withPosition(6, i * 2)
          .getEntry();
      swerveModuleShuffleTargetSpeed[i] = swerveTab.add("M" + i + " TarSpeed", 0)
          .withSize(3, 2)
          .withPosition(12, i * 2)
          .getEntry();
      swerveModuleShuffleActualAngle[i] = swerveTab.add("M" + i + " ActAngle", 0)
          .withSize(3, 2)
          .withPosition(9, i * 2)
          .getEntry();
      swerveModuleShuffleActualSpeed[i] = swerveTab.add("M" + i + " ActSpeed", 0)
          .withSize(3, 2)
          .withPosition(15, i * 2)
          .getEntry();
    }
  }
}
