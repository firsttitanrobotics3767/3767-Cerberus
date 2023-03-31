package frc.robot.commands.Pivot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot;

public class supplyPivotSpeed extends CommandBase{
    private final Pivot pivot;
    private final Supplier<Double> speed;

    public supplyPivotSpeed(Supplier<Double> speed, Pivot pivot) {
        this.speed = speed;
        this.pivot = pivot;
        addRequirements(pivot);
    }

    @Override
    public void execute() {
        pivot.setPivotVolts(speed.get() * 10);
    }

    @Override
    public void end(boolean isInterrupted) {
        pivot.setPivotVolts(0);
    }
}