package frc.robot.subsystems;

import static frc.robot.Constants.CoralArmGripperConstants.gripperMotorCurrentLimit;
import static frc.robot.Constants.CoralArmGripperConstants.gripperMotorID;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralArmGripper extends SubsystemBase {
    private static CoralArmGripper instance;

    private static final TalonFX gripper_motor = new TalonFX(gripperMotorID, "canivore");

    private CoralArmGripper() {
        configureMotors();
    }

    public static class GripperVoltage {
        public static final double zero = 0;
        public static final double intakeCoral = 2.5; // testing higher voltage
        public static final double releaseCoralL4 = -0.5;
        public static final double releaseCoralL3 = -1.5;
        public static final double releaseCoralL2 = -1.5;
        public static final double releaseCoralL1 = -1.5;
        public static final double holdAlgae = 1;
        public static final double extractAlgae = 10;
        public static final double scoreAlgaeProcessor = -6;
        public static final double scoreAlgaeBarge = -12;
    }

    public static synchronized CoralArmGripper getInstance() {
        if (instance == null) {
            instance = new CoralArmGripper();
        }

        return instance;
    }

    private void configureMotors() {
        var gripper_cfg = new TalonFXConfiguration();
        gripper_cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        gripper_cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        gripper_cfg.CurrentLimits.SupplyCurrentLimit = gripperMotorCurrentLimit;
    }

    // Gripper
    public Command setGripperVoltage(double voltage) {
        return run(() -> gripper_motor.setControl(new VoltageOut(voltage)));
    }
}