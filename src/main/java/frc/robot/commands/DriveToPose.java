package frc.robot.commands;

import java.lang.annotation.Target;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.GeomUtil;

public class DriveToPose extends Command {
    private static final double drivekP = 2; // in m/s, so 6 is unreasonable
    private static final double drivekD = 0.0;
    private static final double thetakP = 4.0;
    private static final double thetakD = 0.0;
    private static final double driveMaxVelocity = 0.55;
    private static final double driveMaxAcceleration = 1;
    private static final double thetaMaxVelocity = Units.degreesToRadians(360.0);
    private static final double thetaMaxAcceleration = 8.0;
    public static final double driveTolerance = 0.01;
    public static final double thetaTolerance = Units.degreesToRadians(1.0);
    private static final double ffMinRadius = 0;
    private static final double ffMaxRadius = 0.05;

    private CommandSwerveDrivetrain chassis;
    private Supplier<Pose2d> target;
    private final SwerveRequest.ApplyRobotSpeeds drive = new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity);

    private static final ProfiledPIDController driveController = new ProfiledPIDController(drivekP, 0.0, drivekD,
        new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration));
    private static final ProfiledPIDController thetaController = new ProfiledPIDController(thetakP, 0.0, thetakD,
        new TrapezoidProfile.Constraints(thetaMaxVelocity, thetaMaxAcceleration));
    private Translation2d lastSetpointTranslation = Translation2d.kZero;
    private double driveErrorAbs = 0.01;
    private double thetaErrorAbs = 0.0;
    private boolean running = false;
    private Supplier<Pose2d> robot = () -> chassis.getPose();

    private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
    private DoubleSupplier omegaFF = () -> 0.0;

    private Field2d targetField2d = new Field2d();

    public DriveToPose(CommandSwerveDrivetrain chassis, Supplier<Pose2d> target) {
        this.chassis = chassis;
        this.target = target;

        driveController.setTolerance(driveTolerance);
        thetaController.setTolerance(thetaTolerance);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        // SmartDashboard.putString("init", target.get().toString()); // Diagnostic
        SmartDashboard.putData("targetPose", targetField2d); // Diagnostic
        targetField2d.setRobotPose(target.get());
        Pose2d currentPose = robot.get();
        ChassisSpeeds fieldVelocity = chassis.getState().Speeds;
        Translation2d linearFieldVelocity =
            new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
        driveController.reset(
            currentPose.getTranslation().getDistance(target.get().getTranslation()),
            Math.min(
                0.0,
                -linearFieldVelocity
                    .rotateBy(
                        target
                            .get()
                            .getTranslation()
                            .minus(currentPose.getTranslation())
                            .getAngle()
                            .unaryMinus())
                    .getX()));
        thetaController.reset(
            currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
        lastSetpointTranslation = currentPose.getTranslation();
    }

    @Override
    public void execute() {
        running = true;

        // Get current pose and target pose
        Pose2d currentPose = robot.get();
        Pose2d targetPose = target.get();

        // Calculate drive speed
        double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double ffScaler =
            MathUtil.clamp(
                (currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius),
                0.0,
                1.0);
        driveErrorAbs = currentDistance;
        driveController.reset(
            lastSetpointTranslation.getDistance(targetPose.getTranslation()),
            driveController.getSetpoint().velocity);
        double driveVelocityScalar =
            driveController.getSetpoint().velocity * ffScaler
                + driveController.calculate(driveErrorAbs, 0.0);
        if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;
        lastSetpointTranslation =
            new Pose2d(
                    targetPose.getTranslation(),
                    new Rotation2d(
                        Math.atan2(
                            currentPose.getTranslation().getY() - targetPose.getTranslation().getY(),
                            currentPose.getTranslation().getX() - targetPose.getTranslation().getX())))
                .transformBy(GeomUtil.toTransform2d(driveController.getSetpoint().position, 0.0))
                .getTranslation();

        // Calculate theta speed
        double thetaVelocity =
            thetaController.getSetpoint().velocity * ffScaler
                + thetaController.calculate(
                    currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
        thetaErrorAbs =
            Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
        if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

        Translation2d driveVelocity =
            new Pose2d(
                    Translation2d.kZero,
                    new Rotation2d(
                        Math.atan2(
                            currentPose.getTranslation().getY() - targetPose.getTranslation().getY(),
                            currentPose.getTranslation().getX() - targetPose.getTranslation().getX())))
                .transformBy(GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
                .getTranslation();

        // Scale feedback velocities by input ff
        final double linearS = linearFF.get().getNorm() * 3.0;
        final double thetaS = Math.abs(omegaFF.getAsDouble()) * 3.0;
        driveVelocity =
            driveVelocity.interpolate(linearFF.get().times(4.69), linearS);
        thetaVelocity =
            MathUtil.interpolate(
                thetaVelocity, omegaFF.getAsDouble() * 4.69/Math.hypot(10.75, 10.75), thetaS);

        // Command speeds (ROBOT RELATIVE)
        chassis.setControl(drive.withSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation())));
    }

    @Override
    public void end(boolean interrupted) {
        chassis.setControl(new SwerveRequest.SwerveDriveBrake());
        running = false;
    }

    public boolean atGoal() {
        return running && ((driveController.atGoal() && thetaController.atGoal()) || withinTolerance(driveTolerance, thetaTolerance));
    }

    public boolean withinTolerance(double driveTolerance, double thetaTolerance) {
        return running
            && Math.abs(driveErrorAbs) < driveTolerance
            && Math.abs(thetaErrorAbs) < thetaTolerance;
    }

    public boolean withinTolerance() {
        return running && Math.abs(driveErrorAbs) < driveTolerance && Math.abs(thetaErrorAbs) < thetaTolerance;
    }

}
