package frc.robot.utils;

/**
 * Holds all of the Ids like CAN, pneumatic hub, and DIO devices
 */
public class IDMap {
    /**
     * CAN Ids
     */
    public enum CAN {
        // Drivetrain
        leftFront(1),
        leftBack(11),
        rightFront(2),
        rightBack(21),

        // Pivot
        pivot(3),

        // Arm
        arm(4);

        public final int ID;
        private CAN(int id) {
            ID = id;
        }
    }

    /**
     * Pneumatic hub ports for solenoids
     */
    public enum Pneumatics {
        // Manipulator
        clawOpen(15),
        clawClose(6),

        wristUp(7),
        wristDown(14),

        // cone driver
        coneDriverForward(0),

        coneDriverReverse(1);
        public final int port;
        private Pneumatics(int p) {
            port = p;
        }
    }

    public enum DIO {
        // Drivetrain
        leftDriveEncoderA(0),
        leftDriveEncoderB(1),
        rightDriveEncoderA(2),
        rightDriveEncoderB(3),

        // Pivot
        pivotEncoderA(4),
        pivotEncoderB(5),
        pivotForwardLimit(6),
        pivotReverseLimit(7),

        // Arm
        armEncoderA(8),
        armEncoderB(9),
        armForawrdLimit(10),
        armReverseLimit(11);

        public final int port;
        private DIO(int p) {
            port = p;
        }

    }
}
