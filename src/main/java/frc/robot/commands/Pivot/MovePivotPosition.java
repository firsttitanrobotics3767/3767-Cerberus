package frc.robot.commands.Pivot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot;

public class MovePivotPosition extends CommandBase{
    private final Pivot pivot;
    private final Supplier<Double> speed;

    public MovePivotPosition(Pivot pivot, Supplier<Double> speed) {
        this.pivot = pivot;
        this.speed = speed;
    }

    @Override
    public void execute() {
        pivot.addPivotPosition(speed.get());
    }
}
