package frc.robot.subsystems;

import static frc.robot.Constants.L1PivotConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class L1Pivot extends SubsystemBase {
    private static L1Pivot instance;

    private static final TalonFX pivotMotor = new TalonFX(pivotMotorID, "canivore");
    VoltageOut voltage_out = new VoltageOut(0);
    private static final DigitalInput lim_switch = new DigitalInput(limSwitchDIO);
    public final Trigger lim_switch_triggered = new Trigger(lim_switch::get);

    L1Pivot() {
        configureMotors();

        pivotMotor.setPosition(PivotAngle.storage);
        lim_switch_triggered.onTrue(Commands.runOnce(() -> pivotMotor.setPosition(PivotAngle.storage)));
    }

    public static class PivotAngle {
        public static final double storage = -0.249268;
        public static final double intake = 0;
        public static final double score = -0.249268;

    }

    public static synchronized L1Pivot getInstance() {
        if (instance == null) {
            instance = new L1Pivot();
        }

        return instance;
    }

    public double getPivotAngle() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    private void configureMotors() {
        var pivot_cfg = new TalonFXConfiguration();

        // Use internal sensor as feedback source
        pivot_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        pivot_cfg.Feedback.SensorToMechanismRatio = pivotMotorGearRatio;
        pivot_cfg.Feedback.FeedbackRotorOffset = 0.244385;

        pivotMotor.getConfigurator().apply(pivot_cfg);
    }

    @Override
    public void periodic() {
        System.out.println(lim_switch.get());
    }

    public Command setVoltage(double voltage) {
        return run(() -> pivotMotor.setControl(voltage_out.withOutput(voltage)));
    }

    public double getPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    public boolean atPosition(double position, double goal, double tolerance) {
        return Math.abs(position - goal) > tolerance;
    }

    CoastOut coast = new CoastOut();
    StaticBrake staticBrake = new StaticBrake();

    public Command setCoast() {
        return runOnce(() -> pivotMotor.setControl(coast));
    }

    public Command setBrake() {
        return runOnce(() -> pivotMotor.setControl(staticBrake));
    }
}