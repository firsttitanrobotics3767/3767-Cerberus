package frc.robot.commands.Arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Constants;

public class supplyArmSpeed extends CommandBase{
    private final Arm arm;
    private final Supplier<Double> speed;
    private double volts;

    public supplyArmSpeed(Supplier<Double> speed, Arm arm) {
        this.speed = speed;
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        volts = speed.get() * 10;
        if (arm.getArmPosition() <= 5) {
            // Clamps the speed to -1 if it is below that when close to retraction limit
            volts = Math.max(Constants.Arm.kLowSpeed, volts);
        }
        arm.setVolts(volts);
    }

    @Override
    public void end(boolean isInterrupted) {
        arm.setVolts(0);
    }
}
