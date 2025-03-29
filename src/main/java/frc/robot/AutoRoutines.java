package frc.robot;

import java.util.function.Supplier;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.supersystems.ElevatorSupersystem;
import frc.robot.util.AllianceFlipUtil;

public class AutoRoutines {
        private final ElevatorSupersystem supersystem = ElevatorSupersystem.getInstance();

        private final AutoFactory factory;
        private final CommandSwerveDrivetrain drivetrain;

        public AutoRoutines(AutoFactory factory, CommandSwerveDrivetrain drivetrain) {
                this.factory = factory;
                this.drivetrain = drivetrain;
        }

        public AutoRoutine Center1l4() {
                final AutoRoutine routine = factory.newRoutine("Center 1L4");
                final AutoTrajectory path = routine.trajectory("center_1l4", 0);
                final AutoTrajectory driveback = routine.trajectory("center_1l4", 1);

                routine.active().onTrue(Commands.sequence(
                                path.resetOdometry(),
                                path.cmd()));

                path.atTime("Storage Position").onTrue(supersystem.storagePosition());
                path.atTime("Prepare L4").onTrue(supersystem.coralPrepareL4());

                path.atTime("Score L4")
                                .onTrue(Commands.waitUntil(supersystem.canScoreL4).andThen(supersystem.coralScoreL4()));

                path.recentlyDone().and(supersystem.hasScoredL4).onTrue(driveback.cmd());
                driveback.recentlyDone().onTrue(supersystem.storagePosition());

                return routine;
        }

        public AutoRoutine BlueCenterCage2l4() {
                final AutoRoutine routine = factory.newRoutine("Blue Center Cage 2L4");
                final AutoTrajectory drive_to_1l4 = routine.trajectory("blue_centercage_2l4", 0);
                final AutoTrajectory drive_to_1hps = routine.trajectory("blue_centercage_2l4", 1);
                final AutoTrajectory drive_to_2l4 = routine.trajectory("blue_centercage_2l4", 2);
                final AutoTrajectory driveback = routine.trajectory("blue_centercage_2l4", 3);

                routine.active().onTrue(Commands.sequence(
                                drive_to_1l4.resetOdometry(),
                                drive_to_1l4.cmd()));

                drive_to_1l4.atTime("Storage Position").onTrue(supersystem.storagePosition());
                drive_to_1l4.atTime("Prepare L4").onTrue(supersystem.coralPrepareL4());
                drive_to_1l4.atTime("Score L4")
                                .onTrue(Commands.waitUntil(supersystem.canScoreL4).andThen(supersystem.coralScoreL4()));

                drive_to_1l4.recentlyDone().and(supersystem.hasScoredL4).onTrue(drive_to_1hps.cmd());

                drive_to_1hps.atTime("Prepare Intake").onTrue(supersystem.intakePrepare());
                drive_to_1hps.recentlyDone().onTrue(
                                Commands.waitSeconds(1.5) // Time for HP to place coral
                                                .andThen(supersystem.intakeLoad()
                                                                .until(supersystem.hasIntaked)
                                                                .withTimeout(5))
                                                .andThen(drive_to_2l4.spawnCmd()));

                drive_to_2l4.atTime("Prepare L4 2").and(supersystem.hasCoral).onTrue(supersystem.coralPrepareL4());
                drive_to_2l4.atTime("Score L4 2").and(supersystem.hasCoral)
                                .onTrue(Commands.waitUntil(supersystem.canScoreL4).andThen(supersystem.coralScoreL4()));

                drive_to_2l4.recentlyDone().and(supersystem.hasScoredL4).onTrue(driveback.cmd());

                driveback.atTime("Storage Position 2").onTrue(supersystem.storagePosition());
                driveback.recentlyDone().onTrue(supersystem.storagePosition());

                return routine;
        }

        public AutoRoutine BlueCenterCage2l4AUTOALIGN() {
                final AutoRoutine routine = factory.newRoutine("Blue Center Cage 2L4 AUTOALIGN");
                final AutoTrajectory drive_to_1l4 = routine.trajectory("blue_centercage_2l4_AUTOALIGN", 0);
                final AutoTrajectory drive_to_1hps = routine.trajectory("blue_centercage_2l4_AUTOALIGN", 1);
                final AutoTrajectory drive_to_2l4 = routine.trajectory("blue_centercage_2l4_AUTOALIGN", 2);

                routine.active().onTrue(Commands.sequence(
                                drive_to_1l4.resetOdometry(),
                                drive_to_1l4.cmd()));

                drive_to_1l4.atTime("Storage Position").onTrue(supersystem.storagePosition());
                drive_to_1l4.atTime("Prepare L4").onTrue(supersystem.coralPrepareL4());
                drive_to_1l4.atTime("Score L4")
                                .onTrue(Commands.waitUntil(supersystem.canScoreL4).andThen(supersystem.coralScoreL4()));

                drive_to_1l4.recentlyDone().and(supersystem.hasScoredL4).onTrue(drive_to_1hps.cmd());

                // Supplier<Pose2d> goal_pose = () -> AllianceFlipUtil.apply(new Pose2d(3.982771873474121, 2.8459954261779785, new Rotation2d(1.034896238468816)));
                Supplier<Pose2d> goal_pose = () -> AllianceFlipUtil.apply(new Pose2d(4.08438777923584, 2.9911606311798096, new Rotation2d(1.034896238468816)));

                drive_to_1hps.atTime("Prepare Intake").onTrue(supersystem.intakePrepare());
                drive_to_1hps.recentlyDone().onTrue(
                                Commands.waitSeconds(1.5) // Time for HP to place coral
                                                .andThen(supersystem.intakeLoad()
                                                                .until(supersystem.hasIntaked)
                                                                .withTimeout(5))
                                                .andThen(drive_to_2l4.spawnCmd()));

                drive_to_2l4.recentlyDone().onTrue(
                        new DriveToPose(drivetrain, goal_pose)
                                .andThen(supersystem.coralScoreL4()));

                return routine;
        }

        public AutoRoutine DriveForward() {
                final AutoRoutine routine = factory.newRoutine("Drive Forward");
                final AutoTrajectory path = routine.trajectory("drive_forward");

                routine.active().onTrue(Commands.sequence(
                                path.resetOdometry(),
                                path.cmd()));

                path.atTime("Storage Position").onTrue(supersystem.storagePosition());

                return routine;
        }
}
