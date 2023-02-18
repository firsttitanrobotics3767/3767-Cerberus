package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.Drivetrain.ArcadeDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pivot;

public class RobotContainer {

  private final Drivetrain drivetrain = new Drivetrain();
  private final Pivot pivot = new Pivot();
  private final Arm arm = new Arm();

  private final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);

  public RobotContainer() {
    configureBindings();
    pivot.run(() -> pivot.setPivotSpeed(operator.getRawAxis(1) / 10));
    arm.run(() -> arm.setArmSpeed(operator.getRawAxis(5) / 10));
    drivetrain.setDefaultCommand(new ArcadeDrive(() -> driver.getRawAxis(1), () -> driver.getRawAxis(2), drivetrain));
  }

  private void configureBindings() {
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
