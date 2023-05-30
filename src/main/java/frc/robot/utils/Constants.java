package frc.robot.utils;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    // Camera
    public static final String cameraName = "OV5647";
    /** The translation from the center of the robot, used in vision localization */
    //TODO: calculate transform once camera is mounted
    public static final Transform2d cameraTransform = new Transform2d(new Translation2d(Units.inchesToMeters(3), 0), Rotation2d.fromDegrees(0));

    public static class Drivetrain {
        public static final double throttleLimiter = 0.9;
        public static final double turnLimiter = 0.5;

        //TODO: calculate trackwidth
        public static final double trackWidthMeters = 0.65347;
        public static final double wheelDiameter = Units.inchesToMeters(6.25);
        private static final double countsPerRev = 42.0;
        /* The gyro is producing innaccurate values, so this coefficient is applied in the getGyroYaw method */
        public static final double gyroAdjustment = 0.92783505;

        // 11:62    18:34
        private static final double gearRatio = 10.64;   // 62(34/18)/11
        public static final double metersPerRev = 0.0468876798;

        public static final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(trackWidthMeters);

        public static final PathConstraints pathConstraints = new PathConstraints(1, 5);

        public static class Drive {
            public static final double kP = 0.04;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kS = 0.145;
            public static final double kV = 1;

            public static final double kMaxDriveVel = 40;
            public static final double kSlowVel = 10;
            public static final double kMaxDriveAccel = 200;
        }

        public static class Turn {
            public static final double kP = 0.01;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kS = 0.2;
            public static final double kV = 1;

            /* degrees per second */
            public static final double kMaxTurnVel = 180;
            public static final double kMaxTurnAccel = 720;
        }

        public static class Trajectory {
            public static final double kS = 0.2;
            public static final double kV = 2;
            public static final double kLeftP = 0.1;
            public static final double kRightP = 0;
        }
        
    }

    public static class Pivot {
        public static final double kG = 0.19;
        public static final double kP = 0.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        /** degrees per second squared */
        public static final double kMaxAccel = 500;
        /** degrees per second */
        public static final double kMaxVel = 100;
        // public static final double degreesPerRevolution = 1.8;     // Built in encoder
        public static final double degreesPerRevolution = 360;
    }

    public static class Manipulator {
        public static final int LEDLength = 17;
    }

    public static class Arm {
        public static final double kG = 0.01;
        public static final double kP = 1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        /** Encoder revolutions per second squared */
        public static final double kMaxAccel = 500;
        /** Encoder revolution per second */
        public static final double kMaxVel = 100;
        // (1 / 8192) * 4 * 12 * (30/18)
        // this should be exactly the same distance that was read from the built in motor encoder
        public static final double kDistancePerPulse = 0.009765625;
        public static final double kLowSpeed = -1;
    }
}
