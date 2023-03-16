package frc.robot.commands.auton.balance;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Drivetrain;

public class ForwardBalance extends SequentialCommandGroup {
    private boolean onChargingStation = false;
    /**
     * <p><b>Start:</b> Facing charge station
     * 
     * <p><b>Score:</b> Engage
     */
    public ForwardBalance(Drivetrain drivetrain) {
        addRequirements(drivetrain);
        addCommands(
            new InstantCommand(() -> drivetrain.arcadeDrive(0.35, 0)),
            new WaitUntilCommand(() -> (drivetrain.getGyroPitch() > 13)),
            new InstantCommand(() -> onChargingStation = true),
            new WaitCommand(3.5),
            new InstantCommand(() -> drivetrain.arcadeDrive(0.26, 0)),
            new WaitUntilCommand(() -> onChargingStation && (drivetrain.getGyroPitch() < 10 && drivetrain.getGyroPitch() > -10)),
            new InstantCommand(() -> drivetrain.arcadeDrive(-0.3, 0)),
            new WaitCommand(0.1),
            new InstantCommand(() -> drivetrain.arcadeDrive(0, 0))
            
        );
    }

    
}