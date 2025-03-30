package frc.robot.supersystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.L1Gripper;
import frc.robot.subsystems.L1Gripper.GripperVoltage;
import frc.robot.subsystems.L1Pivot;
import frc.robot.subsystems.L1Pivot.PivotAngle;

public class L1Supersystem {
    private static L1Supersystem instance;

    public static final L1Gripper l1_gripper = L1Gripper.getInstance();
    public static final L1Pivot l1_pivot = L1Pivot.getInstance();

    public Command setStateGripper(double gripper_voltage) {
        return Commands.parallel(
                l1_gripper.setGripperVoltage(gripper_voltage));
    }

    public static synchronized L1Supersystem getInstance() {
        if (instance == null) {
            instance = new L1Supersystem();
        }

        return instance;
    }

    public Command deployIntake() {
        return Commands.parallel(
            l1_pivot.setVoltage(1.2),
            l1_gripper.setGripperVoltage(GripperVoltage.intake)
        )
            .until(() -> l1_pivot.atPosition(l1_pivot.getPosition(), L1Pivot.PivotAngle.storage, 0.05))
            .andThen(Commands.parallel(
                l1_pivot.setVoltage(0),
                l1_gripper.setGripperVoltage(GripperVoltage.intake)
            ));
    }

    public Command returnIntake() {
        return Commands.parallel(
            l1_gripper.setGripperVoltage(0),
            l1_pivot.setVoltage(-3)
                // .until(() -> l1_pivot.atPosition(l1_pivot.getPosition(), L1Pivot.PivotAngle.storage, 0.05))
        )
            .until(l1_pivot.lim_switch_triggered)
            .andThen(Commands.parallel(
                l1_gripper.setGripperVoltage(0),
                l1_pivot.setVoltage(0)
            ));
    }

    public Command scoreL1() {
        return l1_gripper.setGripperVoltage(L1Gripper.GripperVoltage.score);
    }

    public Command stopScore() {
        return l1_gripper.setGripperVoltage(L1Gripper.GripperVoltage.zero);
    }

    // public Command scoreL1() {
    //     return setStatePivot(PivotAngle.score)
    //             .until(l1_pivot.isAtAngle(PivotAngle.score))
    //             .andThen(setState(PivotAngle.score, GripperVoltage.score));
    // }

}
