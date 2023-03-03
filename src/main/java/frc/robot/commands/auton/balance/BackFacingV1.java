package frc.robot.commands.auton.balance;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Drivetrain;

public class BackFacingV1 extends SequentialCommandGroup {
    private boolean onChargingStation = false;
    public BackFacingV1(Drivetrain drivetrain) {
        addRequirements(drivetrain);
        addCommands(
            new InstantCommand(() -> drivetrain.arcadeDrive(-0.6, 0)),
            new WaitUntilCommand(() -> (drivetrain.getGyroPitch() > 13)),
            new InstantCommand(() -> drivetrain.arcadeDrive(-0.35, 0)),
            new InstantCommand(() -> onChargingStation = true),
            new WaitUntilCommand(() -> onChargingStation && (drivetrain.getGyroPitch() < 10 && drivetrain.getGyroPitch() > -10)),
            new InstantCommand(() -> onChargingStation = false),
            new WaitCommand(1.4),
            

            new InstantCommand(() -> drivetrain.arcadeDrive(0.35, 0)),
            new WaitUntilCommand(() -> (drivetrain.getGyroPitch() > 13)),
            new InstantCommand(() -> onChargingStation = true),
            new WaitCommand(3.8),
            new InstantCommand(() -> drivetrain.arcadeDrive(0.26, 0)),
            new WaitUntilCommand(() -> onChargingStation && (drivetrain.getGyroPitch() < 10 && drivetrain.getGyroPitch() > -10)),
            new InstantCommand(() -> drivetrain.arcadeDrive(-0.3, 0)),
            new WaitCommand(0.1),
            new InstantCommand(() -> drivetrain.arcadeDrive(0, 0))
            
        );
    }

    
}
/*
public class ForwardFacingV1 extends CommandBase {
    private final Drivetrain drivetrain;
    private boolean onChargingStation = false;

    public ForwardFacingV1(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }
}
*/
