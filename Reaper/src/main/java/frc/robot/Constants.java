// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import java.awt.geom.Point2D;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class ElevatorConstants {
    public static final double kIntakePose = 51.0;
    public static final double kRestPose = 28.0;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static double kCubic = 0.95;
    public static double kLinear = 0.05;
    public static double kDeadband = 0.02;
  }

  public static final class ShooterConstants {
    public static final double kAngleAdjustment = -0.2;

    public static final Point2D[] kAngleTable = {
        new Point2D.Double(45.5, (47.77 + kAngleAdjustment)),
        new Point2D.Double(57.5, (41.76 + kAngleAdjustment)),
        new Point2D.Double(69.5, (35.75 + kAngleAdjustment)),
        new Point2D.Double(87.5, (29.60 + kAngleAdjustment)),
        new Point2D.Double(105.5, (23.80 + kAngleAdjustment)),
        new Point2D.Double(123.5, (18.80 + kAngleAdjustment)),
        new Point2D.Double(141.5, (13.80 + kAngleAdjustment)),
        new Point2D.Double(159.5, (10.90 + kAngleAdjustment)),
        new Point2D.Double(177.5, (8.90 + kAngleAdjustment)),
        new Point2D.Double(195.5, (5.50 + kAngleAdjustment)),
        new Point2D.Double(240.0, (4.50 + kAngleAdjustment))
    };

    public static final Point2D[] kVeloTable = {
        new Point2D.Double(45.5, 37.23),
        new Point2D.Double(57.5, 40.24),
        new Point2D.Double(69.5, 42.81),
        new Point2D.Double(87.5, 47.00),
        new Point2D.Double(105.5, 50.00),
        new Point2D.Double(123.5, 57.00),
        new Point2D.Double(141.5, 68.00),
        new Point2D.Double(159.5, 70.28),
        new Point2D.Double(177.5, 75.00),
        new Point2D.Double(195.5, 75.00),
        new Point2D.Double(240.0, 75.00),

    };

    public static final Point2D[] kTimeTable = {
        new Point2D.Double(1.0, 0.3),
        new Point2D.Double(3.0, 0.35),
        new Point2D.Double(5.0, 0.4),
    };

    public static final Point2D[] kFeedPitch = {
        new Point2D.Double(240.0, 50.0),
        new Point2D.Double(290.0, 38.0),
        new Point2D.Double(350.0, 28.0)
    };

    public static final Point2D[] kFeedVelocity = {
        new Point2D.Double(240.0, 42.0),
        new Point2D.Double(290.0, 45.0),
        new Point2D.Double(350.0, 50.0)
    };

    public static final Point2D[] kFeedTime = {
        new Point2D.Double(6.0, 0.9),
        new Point2D.Double(8.0, 1.0),
        new Point2D.Double(10.0, 1.1)
    };

  }

  public static final class GoalConstants {
    public static final Translation2d kRedGoal = new Translation2d(643.23 / 39.37, 218.42 / 39.37);
    public static final Translation2d kBlueGoal = new Translation2d(8.00 / 39.37, 218.42 / 39.37);
    public static final Translation2d kRedFeed = new Translation2d(610.23 / 39.37, 280.0 / 39.37);
    public static final Translation2d kBlueFeed = new Translation2d(41.0 / 39.37, 280.0 / 39.37);
  }

  /*
   * m_angleTable.put(38.0,(48.07 + m_angleAdjustment));
   * m_angleTable.put(50.0,(42.06 + m_angleAdjustment));
   * m_angleTable.put(62.0,(36.05 + m_angleAdjustment));
   * m_angleTable.put(80.0,(31.76 + m_angleAdjustment));
   * m_angleTable.put(98.0,(26.21 + m_angleAdjustment));
   * m_angleTable.put(116.0,(21.67 + m_angleAdjustment));
   * m_angleTable.put(134.0, (16.95 + m_angleAdjustment));
   * m_angleTable.put(152.0,(13.95 + m_angleAdjustment));
   * m_angleTable.put(178.0,(12.02 + m_angleAdjustment));
   * m_angleTable.put(188.0,(8.58 + m_angleAdjustment));
   * 
   * m_veloTable.put(38.0,37.23);
   * m_veloTable.put(50.0,40.24);
   * m_veloTable.put(62.0,42.81);
   * m_veloTable.put(80.0,45.39);
   * m_veloTable.put(98.0,47.96);
   * m_veloTable.put(116.0,54.83);
   * m_veloTable.put( 134.0, 66.42);
   * m_veloTable.put(152.0,70.28);
   * m_veloTable.put(178.0,78.00);
   * m_veloTable.put(188.0,90.45);
   * 
   * m_timeTable.put(1.0, 0.3);
   * m_timeTable.put(3.0, 0.35);
   * m_timeTable.put(5.0, 0.4);
   * }
   */

  /*
   * public static final class ModuleConstants {
   * private static final double kTranslationGearRatio = 48.0/24.0*30.0/12.0; //
   * Overall gear ratio of the swerve module
   * public static final double kRotationGearRatio = 40.0;
   * private static final double kWheelDiameter = (2.78/39.37)*0.98; // Wheel
   * Diameter in meters, may need to be
   * // experimentally determined due to compliance
   * // of floor/tread material
   * 
   * public static final double kVelocityFactor = (1.0 / kTranslationGearRatio /
   * 60.0) * kWheelDiameter * Math.PI;
   * public static final double kNEOSteerP = 3.0;
   * public static final double[] kTurnPID = { 0.800, 0, 0 }; // Defines the PID
   * values for rotation of the serve
   * // modules, should show some minor oscillation when no
   * // weight is loaded on the modules
   * }
   */

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = 5550.0 / 60.0;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 20) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.05;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 55; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class DriveConstants {

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static double kMaxSpeedMetersPerSecond = 4.25;

    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kTrackWidth = 24.75 / 39.37; // Center distance in meters between right and left
                                                            // wheels on
    // robot
    public static final double kWheelBase = 24.75 / 39.37; // Center distance in meters between front and back
                                                           // wheels on
    // robot

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftTurningCanId = 5;
    public static final int kRearLeftTurningCanId = 7;
    public static final int kFrontRightTurningCanId = 3;
    public static final int kRearRightTurningCanId = 1;

    public static final int kFrontLeftDrivingCanId = 6;
    public static final int kRearLeftDrivingCanId = 8;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearRightDrivingCanId = 2;

    public static final boolean kGyroReversed = false;
    public static final boolean kUseNEO = false;

    public static final int kFrontLeftDriveMotorPort = 1; // CANID of the Translation SparkMAX
    public static final int kFrontRightDriveMotorPort = 3; // CANID of the Translation SparkMAX
    public static final int kBackLeftDriveMotorPort = 5; // CANID of the Translation SparkMAX
    public static final int kBackRightDriveMotorPort = 7; // CANID of the Translation SparkMAX

    public static final int kFrontLeftTurningMotorPort = 2; // CANID of the Rotation SparkMAX
    public static final int kFrontRightTurningMotorPort = 4; // CANID of the Rotation SparkMAX
    public static final int kBackLeftTurningMotorPort = 6; // CANID of the Rotation SparkMAX
    public static final int kBackRightTurningMotorPort = 8; // CANID of the Rotation SparkMAX

    public static final int kFrontLeftTurningEncoderPort = 0; // Analog Port of the Module Absolute Encoder
    public static final int kFrontRightTurningEncoderPort = 1; // Analog Port of the Module Absolute Encoder
    public static final int kBackLeftTurningEncoderPort = 2; // Analog Port of the Module Absolute Encoder
    public static final int kBackRightTurningEncoderPort = 3; // Analog Port of the Module Absolute Encoder

    public static final double kFrontLeftOffset = 2.917;// Encoder Offset in Radians
    public static final double kFrontRightOffset = 3.035; // Encoder Offset in Radians
    public static final double kBackLeftOffset = 4.820; // Encoder Offset in Radians
    public static final double kBackRightOffset = 0.6675 - Math.PI; // Encoder Offset in Radians

    public static final double[] kFrontLeftTuningVals = { 0.015, 0.23, 0.15, 0 }; // {Static Gain, FeedForward,
                                                                                  // Proportional Gain, ModuleID for
                                                                                  // Tuning}
    public static final double[] kFrontRightTuningVals = { 0.015, 0.23, 0.15, 1 }; // {Static Gain, FeedForward,
                                                                                   // Proportional Gain, ModuleID for
                                                                                   // Tuning}
    public static final double[] kBackLeftTuningVals = { 0.015, 0.23, 0.15, 2 }; // {Static Gain, FeedForward,
                                                                                 // Proportional Gain, ModuleID for
                                                                                 // Tuning}
    public static final double[] kBackRightTuningVals = { 0.015, 0.23, 0.15, 3 }; // {Static Gain, FeedForward,
                                                                                  // Proportional Gain, ModuleID for
                                                                                  // Tuning}

    public static final double kWheelBaseRadius = 0.5
        * Math.sqrt(Math.pow(kWheelBase, 2) + Math.pow(kTrackWidth, 2));

    public static final double kMaxAcceleration = 3.0;
    // Conditions & Battery, Robot will not exceed this
    // speed in closed loop control
    public static final double kTestMaxAcceleration = 1.0;
    public static final double kTestMaxSpeedMetersPerSecond = 1.0;

    // but spinning fast is not particularly useful or driver
    // friendly
    public static final double kMaxAngularAccel = 1.5 * Math.PI; // Maximum Angular Speed desired. NOTE: Robot can
                                                                 // exceed this
    // but spinning fast is not particularly useful or driver
    // friendly

    public static final double kInnerDeadband = 0.02; // This value should exceed the maximum value the analog stick may
                                                      // read when not in use (Eliminates "Stick Drift")
    public static final double kOuterDeadband = 0.98; // This value should be lower than the analog stick X or Y reading
                                                      // when aimed at a 45deg angle (Such that X and Y are are
                                                      // maximized simultaneously)

    public static final double[] kKeepAnglePID = { 1.00, 0, 0 }; // Defines the PID values for the keep angle PID

    private static final SwerveModuleState[] kLockedWheelsHelper = kDriveKinematics
        .toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, 1.0));
    public static final SwerveModuleState[] kLockedWheels = {
        new SwerveModuleState(0.0, kLockedWheelsHelper[0].angle.rotateBy(new Rotation2d(Math.PI / 2))),
        new SwerveModuleState(0.0, kLockedWheelsHelper[1].angle.rotateBy(new Rotation2d(Math.PI / 2))),
        new SwerveModuleState(0.0, kLockedWheelsHelper[2].angle.rotateBy(new Rotation2d(Math.PI / 2))),
        new SwerveModuleState(0.0, kLockedWheelsHelper[3].angle.rotateBy(new Rotation2d(Math.PI / 2)))
    };

    // Minimum allowable rotation command (in radians/s) assuming user input is
    // squared using quadraticTransform, this value is always positive and should be
    // compared agaisnt the absolute value of the drive command
    public static final double kMinRotationCommand = DriveConstants.kMaxAngularSpeed
        * Math.pow(DriveConstants.kInnerDeadband, 2);
    // Minimum allowable tranlsation command (in m/s) assuming user input is squared
    // using quadraticTransform, this value is always positive and should be
    // compared agaisnt the absolute value of the drive command
    public static final double kMinTranslationCommand = DriveConstants.kMaxSpeedMetersPerSecond
        * Math.pow(DriveConstants.kInnerDeadband, 2);

  }

  public static final class GlobalConstants {
    public static final double kVoltCompensation = 12.6; // Sets a voltage compensation value ideally 12.6V
    public static final int PCHID = 20;
    public static final int PDHID = 24;
    public static final double kLoopTime = 0.020;
  }

  public static final class ClimberConstants {
    public static final double kforwardSoftlimit = 105.0;
    public static final double kreverseSoftlimit = 0.0;
  }

  public static final class VisionConstants {
    public static final Point2D[] kNoteDistance = {
        new Point2D.Double(-25.35, 114.0),
        new Point2D.Double(-24.35, 104.0),
        new Point2D.Double(-23.20, 94.0),
        new Point2D.Double(-21.42, 84.0),
        new Point2D.Double(-19.65, 74.0),
        new Point2D.Double(-17.19, 64.0),
        new Point2D.Double(-13.58, 54.0), 
        new Point2D.Double(-9.0, 44.0), 
        new Point2D.Double(-0.39, 34.0),
        new Point2D.Double(6.86, 24.0)};

      public static final Translation2d[] kNoteIDs = {
        new Translation2d(8.2706,0.753),
        new Translation2d(8.2706,2.429),
        new Translation2d(8.2706,4.106),
        new Translation2d(8.2706,5.782),
        new Translation2d(8.2706,7.458)
      };

  }

  public static final class Auto {
    public static final HolonomicPathFollowerConfig autoConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(5.0, 0, 0), new PIDConstants(5.0, 0, 0), DriveConstants.kMaxSpeedMetersPerSecond,
        DriveConstants.kWheelBaseRadius, new ReplanningConfig());
  }

}
