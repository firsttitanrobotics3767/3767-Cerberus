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
        // if (-speed.get() > 0.05 && -speed.get() < -0.05) {
        //     pivot.setPivotSpeed(-speed.get());
        // } else {
        //     pivot.positionArm(0);
        // }
        pivot.setPivotSpeed(-speed.get());
    }

    @Override
    public void end(boolean isInterrupted) {
        pivot.setPivotSpeed(0);
    }
}