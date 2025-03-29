package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CoralArmPivotConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class CoralArmPivot extends SubsystemBase {
    private static CoralArmPivot instance;

    private static final TalonFX pivot_motor = new TalonFX(pivotMotorID, "canivore");
    private static final CANcoder pivot_encoder = new CANcoder(pivotEncoderID, "canivore");
    private static final MotionMagicVoltage pivot_mm_voltage = new MotionMagicVoltage(0).withEnableFOC(true);

    private final Trigger atMax = new Trigger(() -> MathUtil.isNear(pivot_motor.getPosition().getValueAsDouble(), 0.23, 0.01));
    private final Trigger atMin = new Trigger(() -> MathUtil.isNear(pivot_motor.getPosition().getValueAsDouble(), -0.23, 0.01));

    private CoralArmPivot() {
        configureMotors();
    }

    public static class PivotAngle {
        public static final double initial = 0.25;
        public static final double storage = 0.15;
        public static final double scoreL1 = 0;
        public static final double scoreL2 = 0.099;
        public static final double scoreL3 = 0.09;
        public static final double scoreL4 = 0.14;
        public static final double extractAlgaeLow = 0;
        public static final double extractAlgaeHigh = 0;
        public static final double intakeCatch = -0.255;
        public static final double postIntakeCatch = -0.255;
        public static final double intakeGrip = -0.255;
        public static final double scoreProcessor = 0;
        public static final double scoreBarge = 0.21;
        public static final double lollipop = -0.03;
    }

    // private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
    //     // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
    //     new SysIdRoutine.Config(
    //         Volts.per(Second).of(0.5),
    //         Volts.of(2),
    //         null,
    //         state -> SignalLogger.writeString("stateA", state.toString())
    //     ),
    //     new SysIdRoutine.Mechanism(
    //         // Tell SysId how to plumb the driving voltage to the motor(s).
    //         output -> {
    //             pivot_motor.setControl(new VoltageOut(output));
    //         },
    //         // Tell SysId how to record a frame of data for each motor on the mechanism being
    //         // characterized.
    //         null,
    //         this
    //     )
    // );

    public static synchronized CoralArmPivot getInstance() {
        if (instance == null) {
            instance = new CoralArmPivot();
        }

        return instance;
    }    

    private void configureMotors() {
        // Encoder
        BaseStatusSignal.setUpdateFrequencyForAll(50, pivot_encoder.getPosition(), pivot_encoder.getVelocity());
        pivot_encoder.optimizeBusUtilization();

        var encoder_cfg = new MagnetSensorConfigs();
        encoder_cfg.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoder_cfg.MagnetOffset = pivotEncoderOffset;
        pivot_encoder.getConfigurator().apply(encoder_cfg);

        // Motor
        BaseStatusSignal.setUpdateFrequencyForAll(50, pivot_motor.getPosition(), pivot_motor.getVelocity(), pivot_motor.getMotorVoltage(), pivot_motor.getRotorVelocity(), pivot_motor.getRotorPosition());
        pivot_motor.optimizeBusUtilization();

        var pivot_cfg = new TalonFXConfiguration();
        pivot_cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivot_cfg.Feedback.FeedbackRemoteSensorID = pivotEncoderID;
        pivot_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        pivot_cfg.Feedback.RotorToSensorRatio = pivotMotorGearRatio;

        pivot_cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        pivot_cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivot_cfg.Slot0.kA = 0.26315;
        pivot_cfg.Slot0.kG = 0.33;
        pivot_cfg.Slot0.kS = 0.10091;
        pivot_cfg.Slot0.kV = 4.4567;
        pivot_cfg.Slot0.kP = 68.82;
        pivot_cfg.Slot0.kI = 0;
        pivot_cfg.Slot0.kD = 8.8037;

        pivot_cfg.MotionMagic.MotionMagicAcceleration = pivotMotorAcceleration;
        pivot_cfg.MotionMagic.MotionMagicCruiseVelocity = pivotMotorCruiseVelocity;

        pivot_cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivot_cfg.CurrentLimits.SupplyCurrentLimit = pivotMotorCurrentLimit;

        pivot_motor.getConfigurator().apply(pivot_cfg);
    }

    private double getAngle() {
        return pivot_encoder.getAbsolutePosition().getValueAsDouble();
    }

    public Trigger isAtAngle(double goalAngle) {
        return new Trigger(() -> MathUtil.isNear(goalAngle, getAngle(), pivotMotorTolerance));
    }

    public Trigger isGreaterThanAngle(double angle) {
        return new Trigger(() -> getAngle() > angle);
    }

    public Command setAngle(double goalAngle) {
        return run(() -> {
            pivot_motor.setControl(
                pivot_mm_voltage.withPosition(goalAngle)
            );
        });
    }

    // public Command runSysICommand() {
    //     return (sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until(atMax))
    //             .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(atMin))
    //             .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(atMax))
    //             .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(atMin))
    //             .andThen(Commands.print("DONE"));
    // }
}