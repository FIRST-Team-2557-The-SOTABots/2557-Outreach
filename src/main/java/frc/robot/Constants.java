// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 5;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(29);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(29);
    // Distance between front and back wheels on robot
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
    public static final int kFrontLeftDrivingCanId = 5;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kFrontRightDrivingCanId = 7;
    public static final int kRearRightDrivingCanId = 1;

    public static final int kFrontLeftTurningCanId = 6;
    public static final int kRearLeftTurningCanId = 4;
    public static final int kFrontRightTurningCanId = 8;
    public static final int kRearRightTurningCanId = 2;

    public static final boolean kGyroReversed = false;
    public static final double kMovementScale = 1;
    public static double kTurningP = 1.5;
    public static double kTurningI = 0;
    public static double kTurningD = 0;

    public static double kDrivingP = 0.0025;
    public static double kDrivingI = 0;
    public static double kDrivingD = 0;
    public static double kDrivingkV = 0;

  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 15;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 20) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05; 
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class FieldConstants {
    public static final class Red {
      public static final Translation2d kHubPosition = new Translation2d(11.92, 4.0);
      public static final Translation2d kShuttleDepot = new Translation2d(14.3, 2.0);
      public static final Translation2d kShuttleOutpost = new Translation2d(14.3, 6.0);
    }
    public static final class Blue {
      public static final Translation2d kHubPosition = new Translation2d(4.63, 4.0);
      public static final Translation2d kShuttleDepot = new Translation2d(2.5, 2.0);
      public static final Translation2d kShuttleOutpost = new Translation2d(2.5, 6.0
      );
    }
  }

  public static final class ShooterConstants {
    public static final int kShooterFlyWheel1CanId = 12;
    public static final int kShooterFlyWheel2CanId = 13;
    public static final int kShooterFlyWheel3CanId = 14;
    public static final int kShooterFlyWheel4CanId = 18;

    /* Feeder, NOT SHOOTER DELIVERY */
    public static final int kShooterDelivery1CanId = 15;
    public static final int kShooterDelivery2CanId = 16;

    
    public static final double kProjectileSpeedMetersPerSecond = 15.0;
    public static final double kShooterDeliveryVoltage = 8.0;

    public static final double kFlyWheelP = .004; 
    public static final double kFlyWheelI = 0;
    public static final double kFlyWheelD = 0.0;
    public static final double kFlyWheelToleranceRPM = 50.0; 

    
    //im porrammging
    /*  
     * for(long i = 0; i < 9223372036854775807; i++){
     * if(i == 9223372036854775806){
     * System.exit(0);
     * }     * 
     * }
     */
    //shhhhhhhh
    public static final InterpolatingDoubleTreeMap kFlywheelRPMMap = new InterpolatingDoubleTreeMap();
    static {
      // Populate the flywheel RPM map with distance (in meters) to flywheel RPM pairs
      kFlywheelRPMMap.put(1.2,3200.0);
      kFlywheelRPMMap.put(1.5,3300.0);
      kFlywheelRPMMap.put(2.0, 3800.0);
      kFlywheelRPMMap.put(2.5, 4100.0);
      kFlywheelRPMMap.put(3.0, 4600.0);
      kFlywheelRPMMap.put(3.5, 5800.0);
      kFlywheelRPMMap.put(4.0, 6200.0);
    }

    public static final InterpolatingDoubleTreeMap kFlywheelMap = new InterpolatingDoubleTreeMap();
    static {
      // Populate the flywheel RPM map with voltage to flywheel RPM pairs  
      kFlywheelMap.put(0.0, 0.0);
      kFlywheelMap.put(1070.0, 2.0);
      kFlywheelMap.put(2175.0, 4.0);
      kFlywheelMap.put(3295.0, 6.0);
      kFlywheelMap.put(4400.0, 8.0);
      kFlywheelMap.put(5505.0, 10.0);
      kFlywheelMap.put(6075.0, 12.0);
    }
  }

  public static final class  IntakeConstants {
    /* CHANGE NAMES!!! */
    public static final int kIntakeIntake2CanId = 17; // Left intake
    public static final int kIntakeIntakeCanId = 10; // Right intake
    public static final int kIntakeROTCanId = 9;

    public static final double kIntakeROTkP = 2;
    public static final double kIntakeROTkI = 0.0;
    public static final double kIntakeROTkD = 0.0;

    public static final double kIntakeRestingPosition = 0.0;
    public static final double kIntakeDeployedPosition = 00;
    public static final double kIntakeClimbingPosition = 0.5;
    public static final double kIntakeTrenchPosition = 0.25;

    public static final class IntakePosition {
      public static final double kGround = 0.57;
      public static final double kStowed = 0.01;
      public static final double kClimbing = 0.5;
      public static final double kTrench = 0.25;
    }

  }

  public static final class DeliveryConstants {
    public static final int kDeliveryCanId = 11; // Hot dog delivery
    public static final double kDeliveryVoltage = -10.0;
  }

  
}
