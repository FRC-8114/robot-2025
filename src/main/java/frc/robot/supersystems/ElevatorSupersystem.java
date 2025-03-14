package frc.robot.supersystems;

import static frc.robot.Constants.ElevatorSupersystemConstants.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Preset;
import frc.robot.subsystems.CoralArmGripper;
import frc.robot.subsystems.CoralArmPivot;
import frc.robot.subsystems.Elevator;

public class ElevatorSupersystem {
    private static ElevatorSupersystem instance;

    private static final Elevator elevator = Elevator.getInstance();
    private static final CoralArmGripper coral_arm_gripper = CoralArmGripper.getInstance();
    private static final CoralArmPivot coral_arm_pivot = CoralArmPivot.getInstance();

    private double cur_elevator_height = 0.0;
    private double cur_arm_angle = 0.0;
    private double cur_gripper_voltage = 0.0;
    public static boolean beam_break_override = false;
    
    public static final DigitalInput beam_break_sensor = new DigitalInput(beamBreakSensorDIO);
    public final Trigger hasCoral = new Trigger(() -> beam_break_sensor.get()).negate();

    public Command setState(double elevator_height, double arm_angle, double gripper_voltage) {
        cur_elevator_height = elevator_height;
        cur_arm_angle = arm_angle;
        cur_gripper_voltage = gripper_voltage;
        return Commands.parallel(
                elevator.setHeight(elevator_height),
                coral_arm_pivot.setAngle(arm_angle),
                coral_arm_gripper.setGripperVoltage(gripper_voltage));
    }

    public Command setState(double elevator_height, double arm_angle) {
        cur_elevator_height = elevator_height;
        cur_arm_angle = arm_angle;
        return setState(elevator_height, arm_angle, cur_gripper_voltage);
    }

    public Command setStateElevator(double elevator_height) {
        cur_elevator_height = elevator_height;
        return setState(elevator_height, cur_arm_angle, cur_gripper_voltage);
    }

    public Command setStateGripper(double gripper_voltage) {
        cur_gripper_voltage = gripper_voltage;
        return setState(cur_elevator_height, cur_arm_angle, gripper_voltage);
    }

    public Command setStatePivot(double arm_angle) {
        cur_arm_angle = arm_angle;
        return setState(cur_elevator_height, arm_angle, cur_gripper_voltage);
    }

    public Command setStatePreset(Preset preset) {
        cur_elevator_height = preset.getHeight();
        cur_arm_angle = preset.getAngle();
        return setState(preset.getHeight(), preset.getAngle(), cur_gripper_voltage);
    }

    public Command setStatePreset(Preset preset, double gripper_voltage) {
        cur_elevator_height = preset.getHeight();
        cur_arm_angle = preset.getAngle();
        cur_gripper_voltage = gripper_voltage;
        return setState(preset.getHeight(), preset.getAngle(), gripper_voltage);
    }

    // public Command setStateFromDashboard() {
    //     return Commands.parallel(
    //             elevator.setHeightFromDashboard(),
    //             coral_arm_pivot.setAngleFromDashboard(),
    //             coral_arm_gripper.setVoltageFromDashboard());
    // }

    public static synchronized ElevatorSupersystem getInstance() {
        if (instance == null) {
            instance = new ElevatorSupersystem();
        }

        return instance;
    }

    public Command storagePosition() {
        return setState(cur_elevator_height, Preset.Storage.getAngle(), 0)
                .until(coral_arm_pivot.isGreaterThanAngle(0.01))
                .andThen(setStatePreset(Preset.Storage));
    }

    public Command storagePositionAlgae() {
        return setState(cur_elevator_height, Preset.Storage.getAngle(), 1)
                .until(coral_arm_pivot.isGreaterThanAngle(0.01))
                .andThen(setStatePreset(Preset.Storage));
    }

    // INTAKE
    public Command intakePrepare() {
        return setStatePivot(0)
                .until(coral_arm_pivot.isAtAngle(0))
                .andThen(setStateElevator(Preset.IntakeCatch.getHeight()))
                .until(elevator.isAtHeight(Preset.IntakeCatch.getHeight()))
                .andThen(setStatePreset(Preset.IntakeCatch, 0))
                .onlyIf(hasCoral.negate()); // only run of we don't already have a coral
    }

    public Command intakeLoad() {
        return setStatePreset(Preset.IntakeGrip, 2)
                .until(hasCoral)
                .withTimeout(3)
                .andThen(setStatePreset(Preset.PostIntakeCatch, 0))
                .onlyIf(elevator.isAtHeight(Preset.PostIntakeCatch.getHeight())
                        .and(coral_arm_pivot.isAtAngle(Preset.PostIntakeCatch.getAngle()))
                        .and(hasCoral.negate())); // don't try to intake with a low elevator
    }

    public Command intakePost() {
        return setStateElevator(Preset.PostIntakeCatch.getHeight())
                .until(elevator.isAtHeight(Preset.PostIntakeCatch.getHeight()))
                .andThen(setStatePivot(Preset.Storage.getAngle()))
                .until(coral_arm_pivot.isAtAngle(Preset.Storage.getAngle()))
                .andThen(setStatePreset(Preset.Storage));
    }

    // SCORE CORAL
    public Command coralPrepareL4() {
        return setStatePivot(Preset.ScoreL4.getAngle())
                .until(coral_arm_pivot.isGreaterThanAngle(0.01))
                .andThen(setStatePreset(Preset.ScoreL4))
                .onlyIf(hasCoral);
    }

    public Command coralPrepareL3() {
        return setStatePivot(Preset.ScoreL3.getAngle())
                .until(coral_arm_pivot.isGreaterThanAngle(0.01))
                .andThen(setStatePreset(Preset.ScoreL3))
                .onlyIf(hasCoral);
    }

    public Command coralPrepareL2() {
        return setStatePivot(Preset.ScoreL2.getAngle())
                .until(coral_arm_pivot.isGreaterThanAngle(0.01))
                .andThen(setStatePreset(Preset.ScoreL2))
                .onlyIf(hasCoral);
    }

    public Command coralPrepareL1() {
        return setStatePivot(0)
                .until(coral_arm_pivot.isAtAngle(0))
                .andThen(setStateElevator(Preset.ScoreL1.getHeight()))
                .onlyIf(hasCoral);
    }

    public Trigger canScoreL4 = elevator.isAtHeight(Preset.ScoreL4.getHeight(), 0.02)
        .and(coral_arm_pivot.isAtAngle(Preset.ScoreL4.getAngle()))
        .and(hasCoral);

    public Command coralScoreL4() {
        return setStatePivot(0)
                .until(coral_arm_pivot.isAtAngle(0))
                .andThen(setStateGripper(-0.5))
                .withTimeout(1)
                .andThen(setStateGripper(0))
                .onlyIf(elevator.isAtHeight(Preset.ScoreL4.getHeight(), 0.02)
                        .and(coral_arm_pivot.isAtAngle(Preset.ScoreL4.getAngle()))
                        .and(hasCoral));
    }

    public Command coralScoreL3() {
        return setState(cur_elevator_height, 0, -1.5)
                .onlyIf(elevator.isAtHeight(Preset.ScoreL3.getHeight(), 0.02)
                        .and(coral_arm_pivot.isAtAngle(Preset.ScoreL3.getAngle()))
                        .and(hasCoral));
    }

    public Command coralScoreL2() {
        return setState(cur_elevator_height, 0, -1.5)
                .onlyIf(elevator.isAtHeight(Preset.ScoreL2.getHeight(), 0.02)
                        .and(coral_arm_pivot.isAtAngle(Preset.ScoreL2.getAngle()))
                        .and(hasCoral));
    }

    public Command coralScoreL1() {
        return setState(cur_elevator_height, 0, -0.5)
                .onlyIf(elevator.isAtHeight(Preset.ScoreL1.getHeight(), 0.02)
                        .and(coral_arm_pivot.isAtAngle(0))
                        .and(hasCoral));
    }

    // EXTRACT ALGAE
    public Command extractionPrepareLow() {
        return setState(cur_elevator_height, Preset.ExtractAlgaeLow.getAngle(), 0)
                .until(coral_arm_pivot.isAtAngle(0))
                .andThen(setStatePreset(Preset.ExtractAlgaeLow))
                .onlyIf(hasCoral.negate()); // can't run if we already have a algae
    }

    public Command extractionPrepareHigh() {
        return setStatePivot(0)
                .until(coral_arm_pivot.isAtAngle(0))
                .andThen(setStatePreset(Preset.ExtractAlgaeHigh))
                .onlyIf(hasCoral.negate());
    }

    public Command extractionExtractLow() {
        return setStateGripper(10)
                .until(hasCoral) // TODO: does the coral trigger the beam break?
                .withTimeout(3)
                .andThen(setStateGripper(algaeHoldVoltage))
                .onlyIf(elevator.isAtHeight(Preset.ExtractAlgaeLow.getHeight())
                        .and(coral_arm_pivot.isAtAngle(Preset.ExtractAlgaeLow.getAngle())));
    }

    public Command extractionExtractHigh() {
        return setStateGripper(10)
                .until(hasCoral)
                .withTimeout(3)
                .andThen(setStateGripper(algaeHoldVoltage))
                .onlyIf(elevator.isAtHeight(Preset.ExtractAlgaeHigh.getHeight())
                        .and(coral_arm_pivot.isAtAngle(Preset.ExtractAlgaeHigh.getAngle())));
    }

    public Command extractionStop() {
        return setStateGripper(0);
    }

    // SCORE ALGAE
    public Command algaePrepareProcessor() {
        return setState(cur_elevator_height, Preset.ScoreProcessor.getAngle(), algaeHoldVoltage)
            .until(coral_arm_pivot.isAtAngle(Preset.ScoreProcessor.getAngle()))
            .andThen(setStatePreset(Preset.ScoreProcessor));
    }

    public Command algaeScoreProcessor() {
        return setStateGripper(-6)
            .withTimeout(1.5);
    }
}
