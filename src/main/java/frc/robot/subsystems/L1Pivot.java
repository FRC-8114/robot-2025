package frc.robot.subsystems;

import static frc.robot.Constants.L1PivotConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class L1Pivot extends SubsystemBase {
    private static L1Pivot instance;

    private static final TalonFX pivotMotor = new TalonFX(pivotMotorID, "canivore");
    private static final MotionMagicVoltage pivot_position_voltage = new MotionMagicVoltage(0).withEnableFOC(true);

    L1Pivot() {
        configureMotors();
        
        pivotMotor.setPosition(PivotAngle.storage);
    }

    public static class PivotAngle {
        public static final double storage = 0;
        public static final double intake = 0;
        public static final double score = 0;
        
    }

    public static synchronized L1Pivot getInstance() {
        if (instance == null) {
            instance = new L1Pivot();
        }

        return instance;
    }

    public double PivotAngle() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    public Trigger isAtAngle(double goalAngle) {
        return new Trigger(() -> MathUtil.isNear(goalAngle, PivotAngle(), 0.01));
    }

    private void configureMotors() { 
        var pivot_cfg = new TalonFXConfiguration();
    
        // Use internal sensor as feedback source
        pivot_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        pivot_cfg.Feedback.SensorToMechanismRatio = pivotMotorGearRatio;
    
        // MotionMagic
        pivot_cfg.MotionMagic.MotionMagicAcceleration = pivotMotorAcceleration;
        pivot_cfg.MotionMagic.MotionMagicCruiseVelocity = pivotMotorCruiseVelocity;
    
        // PID
        pivot_cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivot_cfg.Slot0.kG = 0.1;
        pivot_cfg.Slot0.kP = 1;
        pivot_cfg.Slot0.kI = 0;
        pivot_cfg.Slot0.kD = 0.2;

        pivotMotor.getConfigurator().apply(pivot_cfg);
    }

    public Command setPivotAngle(double goalAngle) {
        return run(() -> pivotMotor.setControl(
            pivot_position_voltage.withPosition(goalAngle)
        ));
    }


    
}