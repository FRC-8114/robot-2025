package frc.robot.supersystems;

import static frc.robot.Constants.ElevatorSupersystemConstants.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Preset;
import frc.robot.subsystems.CoralArmPivot;
import frc.robot.subsystems.CoralArmGripper;
import frc.robot.subsystems.Elevator;

public class ElevatorSupersystem {
    private static ElevatorSupersystem instance;

    private static final Elevator elevator = Elevator.getInstance();
    private static final CoralArmGripper coral_arm_gripper = CoralArmGripper.getInstance();
    private static final CoralArmPivot coral_arm_pivot = CoralArmPivot.getInstance();

    private static final DigitalInput beamBreakSensor = new DigitalInput(beamBreakSensorDIO);

    private ElevatorSupersystem() {
    }

    public static synchronized ElevatorSupersystem getInstance() {
        if (instance == null) {
            instance = new ElevatorSupersystem();
        }

        return instance; 
    }

    public Trigger hasCoral() {
        return new Trigger(() -> beamBreakSensor.get());
    }

    // Intake
    public Command intakeSetupIntake() {
        return Commands.parallel(
            elevator.setHeight(Preset.IntakeCatch.getHeight()),
            coral_arm_pivot.setAngle(Preset.IntakeCatch.getAngle())
        );
    }
    public Command intakeLoadIntake() {
        if (elevator.getHeight() >= Preset.IntakeCatch.getHeight()) {
            return Commands.parallel(
                elevator.setHeight(Preset.IntakeGrip.getHeight()),
                coral_arm_gripper.setGripperVoltage(2)
                    .until(hasCoral())
                    .andThen(coral_arm_gripper.setGripperVoltage(0))
            );
        } else {
            return Commands.none();
        }
    }

    // Score Coral
    public static enum CoralLayer {
        L1, L2, L3, L4;
    
        public Preset toPreset() {
            return switch (this) {
                case L1 -> Preset.ScoreL1;
                case L2 -> Preset.ScoreL2;
                case L3 -> Preset.ScoreL3;
                case L4 -> Preset.ScoreL4;
            };
        }
    };
    public Command coralPrepareElevator(CoralLayer selected_layer) {
        return elevator.setHeight(selected_layer.toPreset().getHeight());
    }

    public Command coralPrepareArm(CoralLayer selected_layer) {
        return coral_arm_pivot.setAngle(selected_layer.toPreset().getAngle());
    }

    public Command coralScoreCoral(CoralLayer selected_layer) {
        if (selected_layer == CoralLayer.L1) {
            // it's at 90deg, driver drives forward while we spin gripper motors negative
            return coral_arm_gripper.setGripperVoltage(-3);
        } else if (selected_layer == CoralLayer.L2) {
            // it's at 60deg. needs to rotate down, then driver drives away
            return coral_arm_pivot.setAngle(Preset.ScoreL2.getAngle() - 0.0277); // rotate down 10deg
        } else if (selected_layer == CoralLayer.L3) {
            return coral_arm_pivot.setAngle(Preset.ScoreL3.getAngle() - 0.0277);
        } else if (selected_layer == CoralLayer.L4) {
            return coral_arm_pivot.setAngle(Preset.ScoreL1.getAngle());
        } else {
            return Commands.none();
        }
    }

    // Extract Algae
    public static enum AlgaeExtractionLayer {
        High, Low;
    
        public Preset toPreset() {
            return switch (this) {
                case High -> Preset.ExtractAlgaeLow;
                case Low  -> Preset.ExtractAlgaeHigh;
            };
        }
    };
    public Command algaeExtractionPrepareElevator(AlgaeExtractionLayer selected_layer) {
        return elevator.setHeight(selected_layer.toPreset().getHeight());
    }

    public Command algaeExtractionPrepareArm() {
        return coral_arm_pivot.setAngle(Preset.ExtractAlgaeLow.getAngle());
    }
    public Command algaeExtractionExtractAlgae() {
        return coral_arm_gripper.setGripperVoltage(-3);
    }
}
