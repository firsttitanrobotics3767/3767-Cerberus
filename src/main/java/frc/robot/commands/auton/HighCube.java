package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Pivot.SetPivotPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Pivot;

public class HighCube extends SequentialCommandGroup {
    public HighCube(Pivot pivot, Arm arm, Manipulator manipulator) {
        addRequirements(pivot, arm, manipulator);
        addCommands(
            new SetPivotPosition(90.0, pivot)
        );
    }
}
