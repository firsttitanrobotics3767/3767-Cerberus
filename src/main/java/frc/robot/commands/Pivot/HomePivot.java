package frc.robot.commands.Pivot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Pivot;

public class HomePivot extends SequentialCommandGroup {
    public HomePivot(Pivot pivot, Arm arm, Manipulator manipulator) {
        addRequirements(pivot);
        setName("Home Pivot");
        addCommands(
            new InstantCommand(() -> {
                pivot.enableSoftLimits(false);
                // TODO: find acceptable limit for arm extension
                manipulator.wristUp();
                pivot.setPivotVolts(1);
                arm.setArmVolts(-1);
            }),
            new WaitCommand(0.5),
            new InstantCommand(() -> pivot.setPivotVolts(-1)),
            new WaitUntilCommand(() -> pivot.forwardLimitSwitch.get()),
            new InstantCommand(() -> {
                pivot.setPivotVolts(0);
                pivot.resetPivotEncoder(-84);
                pivot.setSoftLimits(-84, 20);
                pivot.enableSoftLimits(true);
            })
        );
    }
}
