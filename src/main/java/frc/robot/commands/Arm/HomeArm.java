package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;

public class HomeArm extends SequentialCommandGroup{
    private final Arm arm;

    public HomeArm(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
        setName("Home Arm");
        addCommands(
            new InstantCommand(() -> {arm.enableSoftlimits(false); arm.setArmVolts(1);}),
            new WaitCommand(0.5),
            new InstantCommand(() -> arm.setArmVolts(-1)),
            new WaitUntilCommand(() -> !arm.reverseLimitSwitch.get()),
            new InstantCommand(() -> {
                arm.setArmVolts(0);
                arm.resetArmEncoder();
                arm.setSoftLimits(93, 1);
                arm.enableSoftlimits(true);
            })
        );
    }
}
