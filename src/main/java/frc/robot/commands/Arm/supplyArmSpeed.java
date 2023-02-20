package frc.robot.commands.Arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SupplyArmSpeed extends CommandBase{
    private final Arm arm;
    private final Supplier<Double> speed;

    public SupplyArmSpeed(Supplier<Double> speed, Arm arm) {
        this.speed = speed;
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setArmSpeed(speed.get());
    }

    @Override
    public void end(boolean isInterrupted) {
        arm.setArmSpeed(0);
    }
}
