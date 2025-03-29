package frc.robot.subsystems;

import static frc.robot.Constants.L1GripperConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class L1Gripper extends SubsystemBase {
    private static L1Gripper instance;

    private static final TalonFX gripperMotor = new TalonFX(gripperMotorID, "canivore");

    private L1Gripper() {
        configureMotors();
    }

    public static class GripperVoltage {
        public static final double zero = 0;
        public static final double intake = -1.5;
        public static final double score = 1.5;
        
    }

    public static synchronized L1Gripper getInstance() {
        if (instance == null) {
            instance = new L1Gripper();
        }

        return instance;
    }

    private void configureMotors() {
        var gripper_cfg = new TalonFXConfiguration();
        gripper_cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        gripper_cfg.CurrentLimits.SupplyCurrentLimit = gripperMotorCurrentLimit;
    }

    public Command setGripperVoltage(double voltage) {
        return run(() -> gripperMotor.setControl(new VoltageOut(voltage)));
    }
}