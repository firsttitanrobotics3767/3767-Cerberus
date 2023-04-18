package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.Arm.AlternateHomeArm;
import frc.robot.commands.Arm.HomeArm;
import frc.robot.commands.Arm.SetArmPosition;
import frc.robot.commands.Drivetrain.DriveMeters;
// import frc.robot.commands.Arm.SetArmPosition;
import frc.robot.commands.Pivot.SetPivotPosition;
import frc.robot.commands.auton.balance.ReverseBalance;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Pivot;

public class HighCubeMobilityBalance extends SequentialCommandGroup{
    private final Drivetrain drivetrain;
    private final Pivot pivot;
    private final Arm arm;
    private final Manipulator manipulator;
    private boolean onChargingStation = false;
     /**
     * <p><b>Start:</b> Facing center cube node
     * 
     * <p><b>Score:</b> High cube, mobility, engage
     */
    public HighCubeMobilityBalance(Drivetrain drivetrain, Pivot pivot, Arm arm, Manipulator manipulator) {
        this.drivetrain = drivetrain;
        this.pivot = pivot;
        this.arm = arm;
        this.manipulator = manipulator;
        addRequirements(drivetrain, pivot, arm, manipulator);
        /*addCommands(
            new SequentialCommandGroup(
                new InstantCommand(() -> drivetrain.arcadeDrive(-0.35, 0)),
                new WaitCommand(0.2),
                new InstantCommand(() -> drivetrain.arcadeDrive(0, 0))).alongWith(
            (new SetPivotPosition(0, pivot).alongWith(new SetArmPosition(0, arm))).withTimeout(1.15)),
            (new SetArmPosition(85, arm).withTimeout(1)).alongWith(new InstantCommand(() -> manipulator.wristDown())),
            new InstantCommand(() -> manipulator.openPincher()),
            new WaitCommand(0.2),
            new InstantCommand(() -> manipulator.wristUp()),
            new SetArmPosition(0, arm).withTimeout(1.1),
            new SetPivotPosition(-80, pivot).withTimeout(2).alongWith(
            
            new InstantCommand(() -> drivetrain.arcadeDrive(-0.35, 0))),
            new WaitUntilCommand(() -> (drivetrain.getGyroPitch() < -13)),
            new InstantCommand(() -> onChargingStation = true),
            new WaitCommand(3.6),
            new InstantCommand(() -> drivetrain.arcadeDrive(-0.26, 0)),
            new WaitUntilCommand(() -> onChargingStation && (drivetrain.getGyroPitch() < 10 && drivetrain.getGyroPitch() > -10)),
            new InstantCommand(() -> drivetrain.arcadeDrive(0.3, 0)),
            new WaitCommand(0.1),
            new InstantCommand(() -> drivetrain.arcadeDrive(0, 0))
        );*/

        DriveMeters mobilityDriveCommand = new DriveMeters(-90, drivetrain, 20, 150);
        DriveMeters balanceDriveCommand = new DriveMeters(100, drivetrain, 20, 150);

        addCommands(
            new ParallelCommandGroup(
                new AlternateHomeArm(pivot, arm),
                new InstantCommand(() -> drivetrain.arcadeDrive(-0.4, 0))
                    .andThen(new WaitCommand(0.3))
                    .andThen(new InstantCommand(() -> drivetrain.arcadeDrive(0, 0)))

            ),
            new ParallelCommandGroup(
                new SetPivotPosition(-5, pivot),
                new WaitCommand(0.5).andThen(new SetArmPosition(60, arm))
            ),
            new InstantCommand(() -> {manipulator.wristDown(); manipulator.openPincher();}),
            new WaitCommand(0.2),
            new InstantCommand(() -> manipulator.wristUp()),
            
            new ParallelCommandGroup(
                new SetArmPosition(0, arm),
                new WaitCommand(1).andThen(new SetPivotPosition(-80, pivot)),
                mobilityDriveCommand,
                new WaitUntilCommand(() -> drivetrain.getGyroPitch() < -13)
                    .andThen(
                        new WaitCommand(1.2),
                        new InstantCommand(() -> mobilityDriveCommand.setConstraints(1, 200)),
                        new WaitUntilCommand(() -> drivetrain.getGyroPitch() > 0),
                        new InstantCommand(() -> mobilityDriveCommand.setConstraints(40, 200))
                    )
            ),
            new ParallelRaceGroup(
                balanceDriveCommand,
                new SequentialCommandGroup(
                    new WaitUntilCommand(() -> drivetrain.getGyroPitch() > 13),
                    new InstantCommand(() -> {balanceDriveCommand.setNewGoal(33); balanceDriveCommand.setConstraints(20, 300);}),
                    new WaitUntilCommand(() -> drivetrain.getGyroPitch() < 6)
                )
            ),
            new InstantCommand(() -> drivetrain.arcadeDrive(-0.3, 0))
                    .andThen(new WaitCommand(0.2))
                    .andThen(new InstantCommand(() -> drivetrain.arcadeDrive(0, 0)))
        );
    }
}
