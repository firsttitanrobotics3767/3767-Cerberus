package frc.robot.utils;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ConeDriver;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Pivot;

public class AllRobotSubsystems {
    public final Drivetrain drivetrain;
    public final Pivot pivot;
    public final Arm arm;
    public final Manipulator manipulator;
    public final ConeDriver coneDriver;

    public AllRobotSubsystems(
        Drivetrain drivetrain,
        Pivot pivot,
        Arm arm,
        Manipulator manipulator,
        ConeDriver coneDriver
        ) {
        this.drivetrain = drivetrain;
        this.pivot = pivot;
        this.arm = arm;
        this.manipulator = manipulator;
        this.coneDriver = coneDriver;
    }
}
