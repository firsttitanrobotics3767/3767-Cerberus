package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class GyroBalance extends CommandBase{
    private final Drivetrain drivetrain;
    private final PIDController balancePID;
    double speed = 0;

    public GyroBalance(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
        balancePID = new PIDController(0.022, 0, 0);
    }

    @Override
    public void execute() {
        speed = balancePID.calculate(drivetrain.getGyroPitch(), 0);
        drivetrain.arcadeDrive(-speed, 0);
        SmartDashboard.putNumber("Gyro Pitch", drivetrain.getGyroPitch());
    }

    @Override
    public void end(boolean isInterrupted) {
        drivetrain.arcadeDrive(speed * 1.5, 0);
        
    }

    @Override
    public boolean isFinished() {
        // return drivetrain.getGyroPitch() > -5 && drivetrain.getGyroPitch() < 5;
        // return false;
        return drivetrain.getGyroPitch() > -10 && drivetrain.getGyroPitch() < 10;
    } 
}
