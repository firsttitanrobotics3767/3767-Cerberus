package frc.robot.commands.balance;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.Drivetrain.ArcadeDrive;
import frc.robot.subsystems.Drivetrain;

public class ForwardFacingV1 extends SequentialCommandGroup {
    private boolean onChargingStation = false;
    public ForwardFacingV1(Drivetrain drivetrain) {
        addRequirements(drivetrain);
        addCommands(
            new ArcadeDrive(() -> 0.35, () -> 0.0, drivetrain),
            new WaitUntilCommand(() -> drivetrain.getGyroPitch() > 8),
            new ArcadeDrive(() -> 0.0, () -> 0.0, drivetrain),
            new InstantCommand(() -> onChargingStation = true),
            new WaitUntilCommand(() -> onChargingStation && (drivetrain.getGyroPitch() < 10 && drivetrain.getGyroPitch() > -10)),
            new ArcadeDrive(() -> 0.0, () -> 0.0, drivetrain)
            
        );
    }

    
}
