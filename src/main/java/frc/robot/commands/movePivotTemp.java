package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot;

public class movePivotTemp extends CommandBase{
    Pivot pivot;

    public movePivotTemp(Pivot pivot) {
        this.pivot = pivot;
    }

    @Override
    public void execute() {
    }


}
