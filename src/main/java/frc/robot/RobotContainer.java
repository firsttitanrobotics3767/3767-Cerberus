package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Arm.HomeArm;
import frc.robot.commands.Arm.supplyArmSpeed;
import frc.robot.commands.Drivetrain.ArcadeDrive;
import frc.robot.commands.Pivot.HomePivot;
import frc.robot.commands.Pivot.supplyPivotSpeed;
import frc.robot.commands.balance.ForwardFacingV1;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Pivot;
import frc.robot.utils.Dashboard;

public class RobotContainer {

  private final Drivetrain drivetrain = new Drivetrain();
  private final Pivot pivot = new Pivot();
  private final Arm arm = new Arm();
  private final Manipulator manipulator = new Manipulator();

  private final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);

  public RobotContainer() {
    CameraServer.startAutomaticCapture();
    configureBindings();
    pivot.setDefaultCommand(new supplyPivotSpeed(() -> operator.getRawAxis(1), pivot));
    arm.setDefaultCommand(new supplyArmSpeed(() -> -operator.getRawAxis(5), arm));
    drivetrain.setDefaultCommand(new ArcadeDrive(() -> -driver.getRawAxis(1), () -> -driver.getRawAxis(2), drivetrain));
  }

  private void configureBindings() {
    JoystickButton openPincher = new JoystickButton(operator, 5);
    JoystickButton closePincher = new JoystickButton(operator, 7);
    JoystickButton wristUp = new JoystickButton(operator, 6);
    JoystickButton wristDown = new JoystickButton(operator, 8);
    JoystickButton balance = new JoystickButton(driver, 1);
    JoystickButton test = new JoystickButton(driver, 4);

    openPincher.onTrue(new InstantCommand(() -> manipulator.openPincher()));
    closePincher.onTrue(new InstantCommand(() -> manipulator.closePincher()));
    wristUp.onTrue(new InstantCommand(() -> manipulator.wristUp()));
    wristDown.onTrue(new InstantCommand(() -> manipulator.wristDown()));
    // balance.whileTrue(new GyroBalance(drivetrain).andThen(new WaitCommand(0.3)).andThen(() -> drivetrain.arcadeDrive(0, 0)));
    balance.whileTrue(new ForwardFacingV1(drivetrain));
    test.onTrue(new ArcadeDrive(() -> 0.35, () -> 0.0, drivetrain));
    test.onFalse(new ArcadeDrive(() -> 0.0, () -> 0.0, drivetrain));

    Dashboard.putSendable("Home Pivot", new HomePivot(pivot));
    Dashboard.putSendable("Home Arm", new HomeArm(arm));
    Dashboard.putSendable("Reset Pivot", new InstantCommand(() -> pivot.resetPivotEncoder()).withName("Reset Pivot"));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
