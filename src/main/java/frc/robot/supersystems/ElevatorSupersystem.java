// @formatter:off

package frc.robot.supersystems;

import static frc.robot.Constants.ElevatorSupersystemConstants.beamBreakSensorDIO;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.CoralArmGripper;
import frc.robot.subsystems.CoralArmGripper.GripperVoltage;
import frc.robot.subsystems.CoralArmPivot;
import frc.robot.subsystems.CoralArmPivot.PivotAngle;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorHeight;

public class ElevatorSupersystem {
    private static ElevatorSupersystem instance;

    public static final Elevator elevator = Elevator.getInstance();
    public static final CoralArmGripper coral_arm_gripper = CoralArmGripper.getInstance();
    public static final CoralArmPivot coral_arm_pivot = CoralArmPivot.getInstance();

    public static boolean beam_break_override = false;
    
    public static final DigitalInput beam_break_sensor = new DigitalInput(beamBreakSensorDIO);
    public final Trigger hasCoral = new Trigger(() -> beam_break_sensor.get()).negate();

    private ElevatorSupersystem() {
        hasCoral
            .onTrue(Commands.runOnce(() -> SmartDashboard.putBoolean("Beam Broken", true)))
            .onFalse(Commands.runOnce(() -> SmartDashboard.putBoolean("Beam Broken", false)));
    }

    public Command setState(double elevator_height, double arm_angle, double gripper_voltage) {
        return Commands.parallel(
            elevator.setHeight(elevator_height),
            coral_arm_pivot.setAngle(arm_angle),
            coral_arm_gripper.setGripperVoltage(gripper_voltage)
        );
    }

    public Command setState(double elevator_height, double arm_angle) {
        return Commands.parallel(
            elevator.setHeight(elevator_height),
            coral_arm_pivot.setAngle(arm_angle)
        );
    }

    public Command setStateElevator(double elevator_height) {
        return Commands.parallel(
            elevator.setHeight(elevator_height)
        );
    }

    public Command setStateGripper(double gripper_voltage) {
        return Commands.parallel(
            coral_arm_gripper.setGripperVoltage(gripper_voltage)
        );
    }

    public Command setStatePivot(double arm_angle) {
        return Commands.parallel(
            coral_arm_pivot.setAngle(arm_angle)
        );
    }

    public Command setStatePivotGrip(double arm_angle, double gripper_voltage) {
        return Commands.parallel(
            coral_arm_pivot.setAngle(arm_angle),
            coral_arm_gripper.setGripperVoltage(gripper_voltage)
        );
    }

    public Command setStateElevatorGrip(double elevator_height, double gripper_voltage) {
        return Commands.parallel(
            elevator.setHeight(elevator_height),
            coral_arm_gripper.setGripperVoltage(gripper_voltage)
        );
    }

    public static synchronized ElevatorSupersystem getInstance() {
        if (instance == null) {
            instance = new ElevatorSupersystem();
        }

        return instance;
    }

    public Command storagePosition() {
        return setStatePivotGrip(PivotAngle.storage, GripperVoltage.zero)
            .until(coral_arm_pivot.isGreaterThanAngle(0.01))
            .andThen(setState(ElevatorHeight.storage, PivotAngle.storage));
    }

    public Command storagePositionAlgae() {
        return setStatePivotGrip(PivotAngle.storage, GripperVoltage.holdAlgae)
            .until(coral_arm_pivot.isGreaterThanAngle(0.01))
            .andThen(setState(ElevatorHeight.storage, PivotAngle.storage));
    }

    // INTAKE
    public Command intakePrepare() {
        return setStatePivot(0)
            .until(coral_arm_pivot.isAtAngle(0))
            .andThen(setStateElevator(ElevatorHeight.intakeCatch))
            .until(elevator.isAtHeight(ElevatorHeight.intakeCatch))
            .andThen(setState(ElevatorHeight.intakeCatch, PivotAngle.intakeCatch, GripperVoltage.zero))
            .onlyIf(hasCoral.negate()); // only run of we don't already have a coral
    }

    public Trigger hasIntaked = (elevator.isAtHeight(ElevatorHeight.postIntakeCatch)
        .and(hasCoral)).or(Robot::isSimulation); // lets us run in sim, even without simulating the arm or the elevator

    public Command intakeLoad() {
        return setState(ElevatorHeight.intakeGrip, PivotAngle.intakeGrip, GripperVoltage.intakeCoral)
            .until(hasCoral)
            .withTimeout(5)
            .andThen(setState(ElevatorHeight.postIntakeCatch, PivotAngle.postIntakeCatch, GripperVoltage.zero))
            .onlyIf(elevator.isAtHeight(ElevatorHeight.intakeCatch)
                .and(coral_arm_pivot.isAtAngle(PivotAngle.intakeCatch))
                .and(hasCoral.negate())); // don't try to intake with a low elevator
    }

    public Command intakePost() {
        return setStateElevator(ElevatorHeight.postIntakeCatch)
            .andThen(setStateGripper(0.0))
            .until(elevator.isAtHeight(ElevatorHeight.postIntakeCatch))
            .andThen(setStatePivot(PivotAngle.storage))
            .until(coral_arm_pivot.isAtAngle(PivotAngle.storage))
            .andThen(setState(ElevatorHeight.storage, PivotAngle.storage, 0));
    }

    // SCORE CORAL
    public Command coralPrepareL4() {
        return setStatePivot(PivotAngle.scoreL4)
            .until(coral_arm_pivot.isGreaterThanAngle(0.01))
            .andThen(setState(ElevatorHeight.scoreL4, PivotAngle.scoreL4))
            .onlyIf(hasCoral);
    }

    public Command coralPrepareL3() {
        return setStatePivot(PivotAngle.scoreL3)
            .until(coral_arm_pivot.isGreaterThanAngle(0.01))
            .andThen(setState(ElevatorHeight.scoreL3, PivotAngle.scoreL3))
            .onlyIf(hasCoral);
    }

    public Command coralPrepareL2() {
        return setStatePivot(PivotAngle.scoreL2)
            .until(coral_arm_pivot.isGreaterThanAngle(0.01))
            .andThen(setState(ElevatorHeight.scoreL2, PivotAngle.scoreL2))
            .onlyIf(hasCoral);
    }

    public Command coralPrepareL1() {
        return setStatePivot(0)
            .until(coral_arm_pivot.isAtAngle(0))
            .andThen(setStateElevator(PivotAngle.scoreL1))
            .onlyIf(hasCoral);
    }

    public Trigger canScoreL4 =
        (elevator.isAtHeight(ElevatorHeight.scoreL4, 0.02)
            .and(coral_arm_pivot.isAtAngle(PivotAngle.scoreL4))
            .and(hasCoral))
        .or(Robot::isSimulation); // lets us run in sim, even without simulating the arm or the elevator;

    public Trigger hasScoredL4 =
        (elevator.isAtHeight(ElevatorHeight.scoreL4, 0.02)
            .and(coral_arm_pivot.isAtAngle(0)))
        .or(Robot::isSimulation); // lets us run in sim, even without simulating the arm or the elevator;

    public Command coralScoreL4() {
        return setStatePivot(0)
                .until(coral_arm_pivot.isAtAngle(0))
                .andThen(setStateGripper(-0.5))
                .withTimeout(1)
                .andThen(setStateGripper(GripperVoltage.zero));
                // .onlyIf(elevator.isAtHeight(Preset.ScoreL4.getHeight(), 0.02)
                //         .and(coral_arm_pivot.isAtAngle(Preset.ScoreL4.getAngle()))
                //         .and(hasCoral));
    }

    public Command coralScoreL3() {
        return setStatePivotGrip(0, GripperVoltage.releaseCoralL3)
                .onlyIf(elevator.isAtHeight(ElevatorHeight.scoreL3, 0.02)
                        .and(coral_arm_pivot.isAtAngle(PivotAngle.scoreL3))
                        .and(hasCoral));
    }

    public Command coralScoreL2() {
        return setStatePivotGrip(0, GripperVoltage.releaseCoralL2)
                .onlyIf(elevator.isAtHeight(ElevatorHeight.scoreL2, 0.02)
                        .and(coral_arm_pivot.isAtAngle(PivotAngle.scoreL2))
                        .and(hasCoral));
    }

    public Command coralScoreL1() {
        return setStatePivotGrip(0, GripperVoltage.releaseCoralL1)
            .onlyIf(elevator.isAtHeight(ElevatorHeight.scoreL1, 0.02)
                .and(coral_arm_pivot.isAtAngle(0))
                .and(hasCoral));
    }

    // EXTRACT ALGAE
    public Command extractionPrepareLow() {
        return setStatePivotGrip(PivotAngle.extractAlgaeLow, 0)
            .until(coral_arm_pivot.isAtAngle(PivotAngle.extractAlgaeLow))
            .andThen(setState(ElevatorHeight.extractAlgaeLow, PivotAngle.extractAlgaeLow))
            .onlyIf(hasCoral.negate()); // can't run if we already have a algae
    }

    public Command extractionPrepareHigh() {
        return setStatePivot(0)
            .until(coral_arm_pivot.isAtAngle(0))
            .andThen(setState(ElevatorHeight.extractAlgaeHigh, PivotAngle.extractAlgaeHigh))
            .onlyIf(hasCoral.negate());
    }

    public Command extractionExtractLow() {
        return setStateGripper(11)
            .until(hasCoral) // TODO: does the coral trigger the beam break?
            .withTimeout(3)
            .andThen(setStateGripper(GripperVoltage.holdAlgae))
            .onlyIf(elevator.isAtHeight(ElevatorHeight.extractAlgaeLow)
                .and(coral_arm_pivot.isAtAngle(PivotAngle.extractAlgaeLow)));
    }

    public Command extractionExtractHigh() {
        return setStateGripper(10)
            .until(hasCoral)
            .withTimeout(3)
            .andThen(setStateGripper(GripperVoltage.holdAlgae))
            .onlyIf(elevator.isAtHeight(ElevatorHeight.extractAlgaeHigh)
                .and(coral_arm_pivot.isAtAngle(PivotAngle.extractAlgaeHigh)));
    }

    public Command extractionStop() {
        return setStateGripper(0);
    }

    // SCORE ALGAE
    public Command algaePrepareProcessor() {
        return setStatePivotGrip(PivotAngle.scoreProcessor, GripperVoltage.holdAlgae)
            .until(coral_arm_pivot.isAtAngle(PivotAngle.scoreProcessor))
            .andThen(setState(ElevatorHeight.scoreProcessor, PivotAngle.scoreProcessor));
    }

    public Command algaeScoreProcessor() {
        return setStateGripper(-6)
            .withTimeout(1.5);
    }

    public Command algaePrepareBarge() {
        return setStatePivotGrip(PivotAngle.scoreL4, GripperVoltage.holdAlgae)
            .until(coral_arm_pivot.isAtAngle(PivotAngle.scoreL4))
            .andThen(setState(ElevatorHeight.scoreBarge, PivotAngle.scoreL4))
            .until(elevator.isAtHeight(ElevatorHeight.scoreBarge))
            .andThen(setState(ElevatorHeight.scoreBarge, PivotAngle.scoreBarge, GripperVoltage.holdAlgae));
    }

    public Command algaeScoreBarge() {
        return setStateGripper(-12)
            .withTimeout(1)
            .andThen(setStateGripper(0));
    }

    // Lollipop
    // public Command lollipop() {
    //     return setStatePivot()
    // }
}
