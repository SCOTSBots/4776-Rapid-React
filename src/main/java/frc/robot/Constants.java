// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static enum RobotType {
    CompBot,
    PracticeBot
  }

  public static final class ShooterConfiguration{
    public final double shooterRPM, hoodWheelRPM, hoodPosition;

    ShooterConfiguration(double shooterRPM, double hoodWheelRPM, double hoodPosition){
      this.shooterRPM = shooterRPM;
      this.hoodWheelRPM = hoodWheelRPM;
      this.hoodPosition = hoodPosition;
    }
  }

  public static final RobotType robotType = RobotType.CompBot;

  public static final class ConfigConstants{
    public static boolean fullShuffleBoardOutput = false;
    public static boolean hasCamera = true;
  }

  public static final class DriveConstants {
    // Any constants that are not final can and should be update in GenerateConstants
    // Non-final constants are initialized with the values of the practice bot below.


    public static final double driveNormalPercentScale = 0.7;
    public static final double rotNormalRateModifier = 1.25;  
    public static final double driveLowPercentScale = 0.5;
    public static final double rotLowRateModifier = 0.75;    


    public static double drivePercentScale = driveNormalPercentScale;
    public static double rotRateModifier = rotNormalRateModifier;

    public static final int kFrontLeftDriveMotorPort = 44;
    public static final int kRearLeftDriveMotorPort = 3;
    public static final int kFrontRightDriveMotorPort = 31;
    public static final int kRearRightDriveMotorPort = 2;

    public static final int kFrontLeftTurningMotorPort = 13;
    public static final int kRearLeftTurningMotorPort = 5;
    public static final int kFrontRightTurningMotorPort = 6;
    public static final int kRearRightTurningMotorPort = 9;

    public static final int[] kFrontLeftTurningEncoderPorts = new int[] { 0, 1 };
    public static final int[] kRearLeftTurningEncoderPorts = new int[] { 2, 3 };
    public static final int[] kFrontRightTurningEncoderPorts = new int[] { 6, 7 };
    public static final int[] kRearRightTurningEncoderPorts = new int[] { 8, 9 };

    public static final int kFrontLeftTurningAnalogPort = 2;
    public static final int kRearLeftTurningAnalogPort = 1;
    public static final int kFrontRightTurningAnalogPort = 3;
    public static final int kRearRightTurningAnalogPort = 0;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kRearLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kRearRightDriveEncoderReversed = false;

    public static Rotation2d kFrontLeftTurningHome = new Rotation2d(Math.toRadians(7.7));
    public static Rotation2d kRearLeftTurningHome = new Rotation2d(Math.toRadians(-19.2));
    public static Rotation2d kFrontRightTurningHome = new Rotation2d(Math.toRadians(+37.5));
    public static Rotation2d kRearRightTurningHome = new Rotation2d(Math.toRadians(-134.5));

    // Distance between centers of right and left wheels on robot
    public static double kTrackWidth = 0.587375;
    // Distance between front and back wheels on robot
    public static double kWheelBase = 0.47; // actually is 0.4953 and was using 0.587375 021222 ;

    public static SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = true;
    public static final double kMaxSpeedMetersPerSecond = 3.35; // Was 1 021222

  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    public static final int kEncoderCPR = 1024;
    public static final int kTurningEncoderCPR = 415;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) kTurningEncoderCPR;

    public static final double kMaxRPM = 5700;
    public static final double kWheelDiameter = 0.102;
    public static final double kMotorGearsToWheelGears = 6.67;
    public static final double kRevolutionsToMeters = Math.PI * kWheelDiameter / kMotorGearsToWheelGears;
    public static final double kRPMToMetersPerSecond = Math.PI * kWheelDiameter / (60 * kMotorGearsToWheelGears);


    public static final double kPModuleTurningController = 2.5;
    public static final double kDModuleTurningController = 0;

    public static final double kPModuleDriveController = 0.6;
    private static final double kDriveP = 15.0;
    private static final double kDriveI = 0.01;
    private static final double kDriveD = 0.1;
    private static final double kDriveF = 0.2;

    public static final double kFrontLeftTurningEncoderCounts = 2 * Math.PI / 415.1;
    public static final double kFrontRightTurningEncoderCounts = 2 * Math.PI / 415.6;
    public static final double kRearLeftTurningEncoderCounts = 2 * Math.PI / 415.2;
    public static final double kRearRightTurningEncoderCounts = 2 * Math.PI / 415.7;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kManipulatorControllerPort = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1.0;//was 3
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;//was 3
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;//was Pi
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class IntakeConstants {
    public static final int intakePackageMotorPort = 14;
    public static final int intakeMotorPort = 10;
    public static final double unpackagePower = 0.5;
    public static final double packagePower = 0.5;
    public static final double intakePower = 0.75;
  }

  public static final class ClimberConstants {
    public static final int liftMotorLeftPort = 12;
    public static final int liftMotorRightPort = 25;
    public static final int feederMotorLeftPort = 17;
    public static final int feederMotorRightPort = 16;
    public static final int armMotorsPort = 35;
    public static final double climbPower = 0.5;
    public static final double lowerPower = 0.2;
    public static final double armPower = 0.5;
  }

  public static final class IntestineConstants {
    public static final int kIntestineMotorPort = 11; 
    public static final int kShooterFeederMotorPort = 30; 
    public static final double kIntestinePower = -0.95;
  }

  public static final class ShooterConstants {
    public static final int kShooterMotorPort = 19;
    public static final int kShooterSecondaryMotorPort = 36;
    public static final int kHoodWheelMotorPort = 7;
    public static final int kTurretMotorPort = 34; 
    public static final int kHoodMotorPort = 18; 
    public static final int kShooterFeederMotorPort = 30; 

    public static boolean hasSecondary = false;

    //*** NOT USING THESE */
    //public static final ShooterConfiguration shootHigh = new ShooterConfiguration(5100, -10000, .45);//was 4800
    //public static final ShooterConfiguration shootMid = new ShooterConfiguration(1600, -8000, 0);
    //public static final ShooterConfiguration shootLow = new ShooterConfiguration(800, -4000, 0.8);
    //*********************/


    public static final ShooterConfiguration shootAutoHighClose = new ShooterConfiguration(1950, -8500, 0);
    public static final ShooterConfiguration shootAutoClose = new ShooterConfiguration(750, -4000, 0.8);
    //public static final ShooterConfiguration shootAutoHighClose = new ShooterConfiguration(2450, -9000, 0);
    public static final ShooterConfiguration shootAutoOutside = new ShooterConfiguration(2850, -9550, 0.6);
    //public static final ShooterConfiguration shootLongShot = new ShooterConfiguration(4700, -10000, 0.5);

    public static final double kShootHighRPM = 4800;
    public static final double kShootLowRPM = 950;
    //public static final double kShootLowRPM = 1600;
    public static final double kHoodWheelHighRPM = -10000;
    public static final double kHoodWheelLowRPM = 0;
    //public static final double kHoodWheelLowRPM = -8000;
    public static final double kSHOT_TIME = 0.3; //Length of time to run the shooter feeder
    public static final double kFeederHoldPower = 0; //Use zero for brake mode / Negative for active hold.


    public static final int kHoodMaxCounts = 90;
    public static final int kTurretMaxCounts = 1000;
    public static final int kTurretMinCounts = -1000;

    // Shooter SparkMAX PID coefficients
    public static final class Shooter {
      public static final double kP = 0.0005069;
      public static final double kI = 0.00000001;
      public static final double kD = 0;
      public static final double kIz = 0;
      public static final double kFF = 0.0002;
      public static final double kMaxOutput = 1;
      public static final double kMinOutput = -0.5;
      public static final double kmaxRPM = 5500;
      public static final double kTypRPM = 4750;
    }

    // Hood SparkMAX PID coefficients
    public static final class Hood {
      public static final double kP = 0.00015;
      public static final double kI = 0.0000001;
      public static final double kD = 0;
      public static final double kIz = 0;
      public static final double kFF = 0;
      public static final double kMaxOutput = 0.5;
      public static final double kMinOutput = -0.5;
      public static final double kmaxRPM = 8000;
    }

    // Hood Wheel SparkMAX PID coefficients
    public static final double kHoodWheelP = 0.00011;
    public static final double kHoodWheelI = 0.0000001;
    public static final double kHoodWheelD = 0;
    public static final double kHoodWheelIz = 0;
    public static final double kHoodWheelFF = 0.000103;
    public static final double kHoodWheelMaxOutput = 1;
    public static final double kHoodWheelMinOutput = -1;
    public static final double kHoodWheelmaxRPM = 10000;
    public static final double kHoodWheelTypRPM = 8000;

    // Turret SparkMAX PID coefficients
    public static final double kTurretP = 0.0001;
    public static final double kTurretI = 0.0;
    public static final double kTurretD = 0;
    public static final double kTurretIz = 0;
    public static final double kTurretFF = 0;
    public static final double kTurretMaxOutput = 1;
    public static final double kTurretMinOutput = -1;
    public static final double kTurretmaxRPM = 8000;
    public static final double kTurretTypRPM = 0;
  }

  public static RobotType GenerateConstants(RobotType robot) {
    switch (robot) {
        case CompBot: {
          ConfigConstants.hasCamera = false;
          ShooterConstants.hasSecondary = false;

          //Swerve Module Alignment
          DriveConstants.kFrontLeftTurningHome = new Rotation2d(Math.toRadians(+165.1));
          DriveConstants.kRearLeftTurningHome = new Rotation2d(Math.toRadians(-96.7));
          DriveConstants.kFrontRightTurningHome = new Rotation2d(Math.toRadians(+77.7));
          DriveConstants.kRearRightTurningHome = new Rotation2d(Math.toRadians(-156.8));//80.8

          // Distance between centers of right and left wheels on robot
          double kTrackWidth = DriveConstants.kTrackWidth = 0.584;
          // Distance between front and back wheels on robot
          double kWheelBase = DriveConstants.kWheelBase = 0.56; // actually is 0.4953 and was using 0.587375 021222 ;

          DriveConstants.kDriveKinematics = new SwerveDriveKinematics(
              new Translation2d(kWheelBase / 2, kTrackWidth / 2), new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
              new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
              new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        }
          break;

        case PracticeBot: {
          ConfigConstants.hasCamera = false;
          ShooterConstants.hasSecondary = false;

          
          DriveConstants.kFrontLeftTurningHome = new Rotation2d(Math.toRadians(7.7));
          DriveConstants.kRearLeftTurningHome = new Rotation2d(Math.toRadians(-19.2));
          DriveConstants.kFrontRightTurningHome = new Rotation2d(Math.toRadians(+37.5));
          DriveConstants.kRearRightTurningHome = new Rotation2d(Math.toRadians(-58.6));//-134.5

          // Distance between centers of right and left wheels on robot
          double kTrackWidth = DriveConstants.kTrackWidth = 0.587375;
          // Distance between front and back wheels on robot
          double kWheelBase = DriveConstants.kWheelBase = 0.47; // actually is 0.4953 and was using 0.587375 021222 ;

          DriveConstants.kDriveKinematics = new SwerveDriveKinematics(
              new Translation2d(kWheelBase / 2, kTrackWidth / 2), new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
              new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
              new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        }
          break;
      }
      return robot;
    }


  }