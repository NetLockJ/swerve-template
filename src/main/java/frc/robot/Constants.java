// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms                                                                                                                                                                
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
  public static class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 1;
    public static final int OPERATOR_CONTROLLER_PORT = 2;
  }

  public static final class FieldConstants {
    public static final double GRAVITY = 9.81; 

    public static Alliance getAlliance() {
      if (DriverStation.getAlliance().isPresent()) {
        return DriverStation.getAlliance().get();
      }

      return Alliance.Blue;
    }
  }

  public static class SwerveModuleConstants {
    public static final int PIGEON_ID = 4;

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double STEERING_GEAR_RATIO = 1 / 12.8;
    // This is for L1 SDS Modules
    public static final double DRIVE_GEAR_RATIO = 8.14;

    public static final double DRIVE_ROTATION_TO_METER = DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER;
    public static final double STEER_ROTATION_TO_RADIANS = STEERING_GEAR_RATIO * Math.PI * 2d;
    public static final double DRIVE_METERS_PER_MINUTE = DRIVE_ROTATION_TO_METER / 60d;
    public static final double STEER_RADIANS_PER_MINUTE = STEER_ROTATION_TO_RADIANS / 60d;

    // TODO: Tune for modules!
    public static final double MODULE_KP = 0.3048;
    public static final double MODULE_KD = 0.0066806;

    // --------- Front Left Module --------- \\
    public static final int FL_DRIVE_ID = 2;
    public static final int FL_STEER_ID = 3;
    public static final int FL_ABSOLUTE_ENCODER_PORT = 1;
    public static final double FL_OFFSET_RADIANS = Units.rotationsToRadians(-0.322998) + Math.PI / 2;
    public static final boolean FL_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean FL_MOTOR_REVERSED = false;

    // --------- Front Right Module --------- \\
    public static final int FR_DRIVE_ID = 6;
    public static final int FR_STEER_ID = 7;
    public static final int FR_ABSOLUTE_ENCODER_PORT = 5;
    public static final double FR_OFFSET_RADIANS = Units.rotationsToRadians(0.445557) + Math.PI / 2;
    public static final boolean FR_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean FR_MOTOR_REVERSED = false;

    // --------- Back Right Module --------- \\
    public static final int BR_DRIVE_ID = 11;
    public static final int BR_STEER_ID = 12;
    public static final int BR_ABSOLUTE_ENCODER_PORT = 13;
    public static final double BR_OFFSET_RADIANS = Units.rotationsToRadians(-0.09) + Math.PI / 2;
    public static final boolean BR_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean BR_MOTOR_REVERSED = false;

    // --------- Back Left Module --------- \\
    public static final int BL_DRIVE_ID = 9;
    public static final int BL_STEER_ID = 10;
    public static final int BL_ABSOLUTE_ENCODER_PORT = 8;
    // -0.401123
    public static final double BL_OFFSET_RADIANS = Units.rotationsToRadians(-0.401123) + Math.PI / 2;
    public static final boolean BL_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean BL_MOTOR_REVERSED = true;

  }

  public static class DriveConstants {
    // TODO: Update max vel 
    public static final double MAX_MODULE_VELOCITY = 5.21;
    public static final double MAX_ROBOT_VELOCITY = 5.21;
    public static final double MAX_ROBOT_RAD_VELOCITY = 12.0; // Approx. Measured rads/sec

    //TODO: Updated based on robot
    public static final double TRACK_WIDTH = Units.inchesToMeters(15);
    public static final double WHEEL_BASE = Units.inchesToMeters(15);
    // TODO: Set angle offset relative to front of robot
    public static final Rotation2d NAVX_ANGLE_OFFSET = Rotation2d.fromDegrees(0);
    // TODO: For PPLib, max radius of drivebase
    public static final double DRIVE_BASE_RADIUS = Units.inchesToMeters(15);

    public static final class ModuleIndices {
      public static final int FRONT_LEFT = 3;
      public static final int FRONT_RIGHT = 2;
      public static final int REAR_LEFT = 1;
      public static final int REAR_RIGHT = 0; // 12
    }

    //TODO: Change based on robot
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
        new Translation2d(TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0),
        new Translation2d(-TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
        new Translation2d(-TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0));

    public static final double XY_SPEED_LIMIT = 0.5;
    public static final double Z_SPEED_LIMIT = 0.5;
  }

  public static class CommonConstants {
    public static final boolean LOG_INTO_FILE_ENABLED = false;
  }

  public static final class PathPlannerConstants {
    //TODO: Change based on tuning
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(5, 0, 0.2);
    public static final PIDConstants ROTATION_PID = new PIDConstants(5, 0, 0.2);

    public static final HolonomicPathFollowerConfig HOLONOMIC_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
        TRANSLATION_PID,
        ROTATION_PID,
        DriveConstants.MAX_MODULE_VELOCITY,
        DriveConstants.DRIVE_BASE_RADIUS,
        new ReplanningConfig());
  }

  public static final class PoseConstants {

    public static final double kPositionStdDevX = 0.1;
    public static final double kPositionStdDevY = 0.1;
    public static final double kPositionStdDevTheta = 10;
    public static final double kVisionStdDevY = 5;
    public static final double kVisionStdDevX = 5;
    public static final double kVisionStdDevTheta = 500;
  }
}
