package frc.robot.commands;

import org.opencv.core.RotatedRect;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.RotationStyle;

public class DriveCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final XboxController xbox;

    private SlewRateLimiter dsratelimiter = new SlewRateLimiter(4);

    // Auto rotation pid/rate limiter
    private PIDController rotationController = new PIDController(6, 0.0, 0.5);

    private double DRIVE_MULT = 1.0;
    private final double SLOWMODE_MULT = 0.25;

    private enum DriveState {
        Free,
        Locked,
    };

    private DriveState state = DriveState.Free;

    public DriveCommand(SwerveSubsystem swerveSubsystem, XboxController xbox) {
        this.swerveSubsystem = swerveSubsystem;
        this.xbox = xbox;

        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        dsratelimiter.reset(SLOWMODE_MULT);

        addRequirements(swerveSubsystem);
    }

    double clamp(double v, double mi, double ma) {
        return (v < mi) ? mi : (v > ma ? ma : v);
    }

    public Translation2d DeadBand(Translation2d input, double deadzone) {
        double mag = input.getNorm();
        Translation2d norm = input.div(mag);

        if (mag < deadzone) {
            return new Translation2d(0.0, 0.0);
        } else {
            Translation2d result = norm.times((mag - deadzone) / (1.0 - deadzone));
            return new Translation2d(
                    clamp(result.getX(), -1.0, 1.0),
                    clamp(result.getY(), -1.0, 1.0));
        }
    }

    public double DeadBand(double input, double deadband) {
        return Math.abs(input) < deadband ? 0.0 : (input - Math.signum(input) * deadband) / (1.0 - deadband);
    }

    @Override
    public void execute() {
        Translation2d xyRaw = new Translation2d(xbox.getLeftX(), xbox.getLeftY());
        Translation2d xySpeed = DeadBand(xyRaw, 0.15/2.f);
        double zSpeed = DeadBand(xbox.getRightX(), 0.1);
        double xSpeed = xySpeed.getX(); 
        double ySpeed = xySpeed.getY();

        // TODO: Full speed!
        xSpeed *= DriveConstants.XY_SPEED_LIMIT * DriveConstants.MAX_ROBOT_VELOCITY;
        ySpeed *= DriveConstants.XY_SPEED_LIMIT * DriveConstants.MAX_ROBOT_VELOCITY;
        zSpeed *= DriveConstants.Z_SPEED_LIMIT * DriveConstants.MAX_ROBOT_RAD_VELOCITY;

        double dmult = dsratelimiter
                .calculate((DRIVE_MULT - SLOWMODE_MULT) * xbox.getRightTriggerAxis() + SLOWMODE_MULT);
        xSpeed *= dmult;
        ySpeed *= dmult;
        zSpeed *= dmult;

        if (xbox.getXButton()) {
            swerveSubsystem.zeroHeading();
            Translation2d pospose = swerveSubsystem.getPose().getTranslation();
            swerveSubsystem.odometry.resetPosition(swerveSubsystem.getRotation2d(), swerveSubsystem.getModulePositions(), 
                new Pose2d(pospose, new Rotation2d(FieldConstants.getAlliance() == Alliance.Blue ? 0.0 : Math.PI))
            );
        }

        ChassisSpeeds speeds;

        // Drive Non Field Oriented
        if (xbox.getRightBumper()) {
            speeds = new ChassisSpeeds(-xSpeed, ySpeed, -zSpeed);

        } else if (!xbox.getLeftBumper()) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-ySpeed, xSpeed, zSpeed,
                    new Rotation2d(
                            -swerveSubsystem.getRotation2d().rotateBy(DriveConstants.NAVX_ANGLE_OFFSET).getRadians()));
        } else {
            // Normal non-field oriented
            speeds = new ChassisSpeeds(-xSpeed, -ySpeed, zSpeed);
        }

        // State transition logic
        switch (state) {
            case Free:
                break;
            case Locked:
                state = ((xyRaw.getNorm() > 0.15) && !xbox.getBButton()) ? DriveState.Free : DriveState.Locked;
                break;
        }

        // Drive execution logic
        switch (state) {
            case Free:
                SwerveModuleState[] calculatedModuleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds);
                swerveSubsystem.setModules(calculatedModuleStates);
                break;
            case Locked:
                swerveSubsystem.setXstance();
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopDrive();
    }

    public boolean isFinished() {
        return false;
    }
}
