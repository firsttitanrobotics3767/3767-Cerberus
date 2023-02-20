package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class GyroBalance extends CommandBase{
    private final Drivetrain drivetrain;
    private final PIDController balancePID;

    public GyroBalance(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
        balancePID = new PIDController(0, 0, 0);
    }

    @Override
    public void execute() {
        drivetrain.arcadeDrive(balancePID.calculate(drivetrain.getGyroPitch(), 0), 0);
    }

    @Override
    public void end(boolean isInterrupted) {
        drivetrain.arcadeDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        // return drivetrain.getGyroPitch() > -5 && drivetrain.getGyroPitch() < 5;
        return false;
    } 
}
