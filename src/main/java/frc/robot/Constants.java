package frc.robot;

public class Constants {
    public static final class ElevatorConstants {
        public static final int leftMotorID = 20;
        public static final int rightMotorID = 21;

        public static final double motorMaxAcceleration = 8;
        public static final double motorCruiseVelocity = 6;

        public static final double motorGearRatio = 7.75;

        public static final double heightTolerance = 0.0015;

        public static final double sprocketRadius = 0.0254 * 0.905;

        // public static final double canrangeOffset = 0.16;

        public static final int heightSensorID = 22;
        public static final int bottomSensorDIO = 0;
    }

    public static final class CoralArmPivotConstants {
        public static final int pivotMotorID = 30;
        public static final int pivotEncoderID = 31;

        public static final double pivotMotorAcceleration = 1;
        public static final double pivotMotorCruiseVelocity = 0.5;

        public static final double pivotEncoderOffset = 0.4169921875;

        public static final double pivotMotorCurrentLimit = 40;

        public static final double pivotMotorGearRatio = 42.85714286;
        public static final double pivotMotorTolerance = 0.01;
    }

    public static final class CoralArmGripperConstants {
        public static final int gripperMotorID = 32;
        public static final double gripperMotorCurrentLimit = 60;
    }

    public static final class L1PivotConstants {
        public static final int pivotMotorID = 35;
        public static final int limSwitchDIO = 0;
        
        public static final double pivotMotorAcceleration = 6;
        public static final double pivotMotorCruiseVelocity = 2.25;

        public static final double pivotMotorGearRatio = 15.0;
    }
    
    public static final class L1GripperConstants {
        public static final int gripperMotorID = 36;
        
        public static final int gripperMotorCurrentLimit = 15;
    }

    public static final class ClimberConstants {
        public static final int climberMotorID = 40;

        public static final int climberMotorTolerance = 1;

        public static final int climberMotorCurrentLimit = 60;
    }

    public static final class ElevatorSupersystemConstants {
        public static final int beamBreakSensorDIO = 6;
    }

    public static final class StatusLEDConstants {
        public static final int statusLEDPort = 9;
        public static final int statusLEDCount = 23;
    }
}
