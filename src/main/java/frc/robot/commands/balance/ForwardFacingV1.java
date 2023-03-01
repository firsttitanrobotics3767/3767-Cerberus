package frc.robot.commands.balance;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.Drivetrain.ArcadeDrive;
import frc.robot.subsystems.Drivetrain;

public class ForwardFacingV1 extends SequentialCommandGroup {
    private boolean onChargingStation = false;
    public ForwardFacingV1(Drivetrain drivetrain) {
        addRequirements(drivetrain);
        addCommands(
            new InstantCommand(() -> drivetrain.arcadeDrive(0.35, 0)),
            new WaitUntilCommand(() -> (drivetrain.getGyroPitch() > 13)),
            new InstantCommand(() -> onChargingStation = true),
            new WaitCommand(3),
            new InstantCommand(() -> drivetrain.arcadeDrive(0.26, 0)),
            new WaitUntilCommand(() -> onChargingStation && (drivetrain.getGyroPitch() < 10 && drivetrain.getGyroPitch() > -10)),
            new InstantCommand(() -> drivetrain.arcadeDrive(0.0, 0))
            
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
