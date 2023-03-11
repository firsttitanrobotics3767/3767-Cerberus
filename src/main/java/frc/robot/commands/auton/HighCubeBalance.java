package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.Arm.SetArmPosition;
// import frc.robot.commands.Arm.SetArmPosition;
import frc.robot.commands.Pivot.SetPivotPosition;
import frc.robot.commands.auton.balance.ReverseBalance;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Pivot;

public class HighCubeBalance extends SequentialCommandGroup{
    private final Drivetrain drivetrain;
    private final Pivot pivot;
    private final Arm arm;
    private final Manipulator manipulator;
    private boolean onChargingStation = false;
     /**
     * <p><b>Start:</b> Facing cube node
     * 
     * <p><b>Score:</b> High cube, engage
     */
    public HighCubeBalance(Drivetrain drivetrain, Pivot pivot, Arm arm, Manipulator manipulator) {
        this.drivetrain = drivetrain;
        this.pivot = pivot;
        this.arm = arm;
        this.manipulator = manipulator;
        addRequirements(drivetrain, pivot, arm, manipulator);
        addCommands(
            new InstantCommand(() -> drivetrain.arcadeDrive(-0.35, 0)),
            new WaitCommand(0.2),
            new InstantCommand(() -> drivetrain.arcadeDrive(0, 0)),
            (new SetPivotPosition(0, pivot).alongWith(new SetArmPosition(0, arm))).withTimeout(1.15),
            (new SetArmPosition(85, arm).withTimeout(1)).alongWith(new InstantCommand(() -> manipulator.wristDown())),
            new InstantCommand(() -> manipulator.openPincher()),
            new WaitCommand(0.2),
            new InstantCommand(() -> manipulator.wristUp()),
            new SetArmPosition(0, arm).withTimeout(1.1),
            new SetPivotPosition(-80, pivot).withTimeout(3).alongWith(
            
            new InstantCommand(() -> drivetrain.arcadeDrive(-0.35, 0))),
            new WaitUntilCommand(() -> (drivetrain.getGyroPitch() < -13)),
            new InstantCommand(() -> onChargingStation = true),
            new WaitCommand(4),
            new InstantCommand(() -> drivetrain.arcadeDrive(-0.26, 0)),
            new WaitUntilCommand(() -> onChargingStation && (drivetrain.getGyroPitch() < 10 && drivetrain.getGyroPitch() > -10)),
            new InstantCommand(() -> drivetrain.arcadeDrive(0.3, 0)),
            new WaitCommand(0.1),
            new InstantCommand(() -> drivetrain.arcadeDrive(0, 0))
        );
    }
}
