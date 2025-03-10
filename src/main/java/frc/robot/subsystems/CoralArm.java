package frc.robot.subsystems;

import static frc.robot.Constants.CoralArmConstants.*;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CoralArm extends SubsystemBase {
    private static CoralArm instance;

    private static final TalonFX pivot_motor = new TalonFX(pivotMotorID, "canivore");
    private static final CANcoder pivot_encoder = new CANcoder(pivotEncoderID, "canivore");
    private static final MotionMagicVoltage pivot_mm_voltage = new MotionMagicVoltage(0).withEnableFOC(true);

    private final SparkMax gripper_motor = new SparkMax(gripperMotorID, MotorType.kBrushless);

    private static class TuneableConstants {
        private static double kG = 0.06;
        private static double kP = 0.12;
        private static double kI = 0;
        private static double kD = 0.01;

        public static void initDashboard() {
            SmartDashboard.putNumber("CoralArm/kG", kG);
            SmartDashboard.putNumber("CoralArm/kP", kP);
            SmartDashboard.putNumber("CoralArm/kI", kI);
            SmartDashboard.putNumber("CoralArm/kD", kD);
        }
        
        // Method to update constants from SmartDashboard
        public static boolean updateDashboard() {
            double newKG = SmartDashboard.getNumber("CoralArm/kG", kG);
            double newKP = SmartDashboard.getNumber("CoralArm/kP", kP);
            double newKI = SmartDashboard.getNumber("CoralArm/kI", kI);
            double newKD = SmartDashboard.getNumber("CoralArm/kD", kD);
            
            // Check if any values have changed
            boolean changed = newKG != kG|| newKP != kP || newKI != kI || newKD != kD;
            
            // Update stored values
            kG = newKG;
            kP = newKP;
            kI = newKI;
            kD = newKD;
            
            return changed;
        }
    }

    private CoralArm() {
        configureMotors();

        TuneableConstants.initDashboard();
    }

    public static synchronized CoralArm getInstance() {
        if (instance == null) {
            instance = new CoralArm();
        }

        return instance;
    }

    private void configureMotors() {
        // Pivot encoder
        var encoder_cfg = new MagnetSensorConfigs();
        encoder_cfg.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoder_cfg.MagnetOffset = 0;
        pivot_encoder.getConfigurator().apply(encoder_cfg);

        // Pivot motor
        var pivot_cfg = new TalonFXConfiguration();
        pivot_cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivot_cfg.Feedback.FeedbackRemoteSensorID = pivotEncoderID;
        pivot_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        pivot_cfg.Feedback.RotorToSensorRatio = pivotMotorGearRatio;

        pivot_cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivot_cfg.Slot0.kG = TuneableConstants.kG;
        pivot_cfg.Slot0.kP = TuneableConstants.kP;
        pivot_cfg.Slot0.kI = TuneableConstants.kI;
        pivot_cfg.Slot0.kD = TuneableConstants.kD;

        pivot_cfg.MotionMagic.MotionMagicAcceleration = pivotMotorAcceleration;
        pivot_cfg.MotionMagic.MotionMagicCruiseVelocity = pivotMotorCruiseVelocity;

        pivot_cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivot_cfg.CurrentLimits.SupplyCurrentLimit = pivotMotorCurrentLimit;

        pivot_motor.getConfigurator().apply(pivot_cfg);

        // Gripper motor
        var gripper_cfg = new SparkMaxConfig();
        gripper_cfg.idleMode(IdleMode.kBrake);
        gripper_cfg.smartCurrentLimit((int) gripperMotorCurrentLimit);
        gripper_motor.configure(gripper_cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // Gripper
    public Command setGripperVoltage(double voltage) {
        return run(() -> gripper_motor.setVoltage(voltage));
    }

    private double getAngle() {
        return pivot_encoder.getAbsolutePosition().getValueAsDouble();
    }

    public Trigger isAtAngle(double goalAngle) {
        return new Trigger(() -> MathUtil.isNear(goalAngle, getAngle(), pivotMotorTolerance));
    }

    public Command setAngle(double goalAngle) {
        return Commands.run(() -> {
            pivot_mm_voltage.withPosition(goalAngle);
        });
    }

    @Override
    public void periodic() {
        boolean changed = TuneableConstants.updateDashboard();
        if (changed) {
            configureMotors();
            System.out.println("Reconfigured Coral Arm");
        }
    }
}
