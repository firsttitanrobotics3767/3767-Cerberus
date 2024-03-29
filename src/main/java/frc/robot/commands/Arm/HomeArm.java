package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Pivot;

public class HomeArm extends SequentialCommandGroup{
    public HomeArm(Pivot pivot, Arm arm) {
        addRequirements(arm);
        setName("Home Arm");
        addCommands(
            new InstantCommand(() -> {
                arm.enableSoftlimits(false);
                //TODO: get accurate acceptable angle
                arm.setVolts(1);
            }),
            new WaitCommand(0.5),
            new InstantCommand(() -> arm.setVolts(-1)),
            new WaitUntilCommand(() -> arm.getReverseLimitSwitchPressed()),
            new InstantCommand(() -> {
                arm.setVolts(0);
                arm.resetArmEncoder();
                arm.setSoftLimits(93, 1);
                arm.enableSoftlimits(true);
            })
        );
    }
}
