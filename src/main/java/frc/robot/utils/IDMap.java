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
        openPincher(15),
        closePincher(6),

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
        pivotForwardLimit(6),
        pivotReverseLimit(5),

        // Arm
        armForawrdLimit(4),
        armReverseLimit(7),
        armEncoderA(8),
        armEncoderB(9),
        armEncoderI(3),

        // Manipulator
        LEDs(9);

        public final int port;
        private DIO(int p) {
            port = p;
        }

    }
}
