package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.Arm.AlternateHomeArm;
import frc.robot.commands.Arm.HomeArm;
import frc.robot.commands.Arm.SetArmPosition;
import frc.robot.commands.Arm.supplyArmSpeed;
import frc.robot.commands.Drivetrain.DriveMeters;
import frc.robot.commands.Drivetrain.TurnDegrees;
import frc.robot.commands.Pivot.HomePivot;
import frc.robot.commands.Pivot.SetPivotPosition;
import frc.robot.commands.Pivot.supplyPivotSpeed;
import frc.robot.commands.auton.HighCubeMobility;
import frc.robot.commands.auton.HighCubeMobilityBalance;
import frc.robot.commands.auton.HighCubeMobilityReverse;
import frc.robot.commands.auton.HighCube;
import frc.robot.commands.auton.HighCubeBalance;
import frc.robot.commands.auton.balance.ForwardBalance;
import frc.robot.commands.auton.balance.ReverseBalance;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Manipulator;
import frc.robot.utils.Dashboard;

public class RobotContainer {

  public final Drivetrain drivetrain = new Drivetrain();
  public final Pivot pivot = new Pivot();
  public final Arm arm = new Arm();
  public final Manipulator manipulator = new Manipulator();

  public final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  public RobotContainer() {
    pivot.arm = arm;
    arm.pivot = pivot;
    CameraServer.startAutomaticCapture();
    configureBindings();
    pivot.setDefaultCommand(new supplyPivotSpeed(() -> -operator.getRawAxis(1), pivot));
    arm.setDefaultCommand(new supplyArmSpeed(() -> -operator.getRawAxis(5), arm));

    autoChooser.setDefaultOption("Forward balance", new ForwardBalance(drivetrain));
    autoChooser.addOption("Empty", new InstantCommand());
    autoChooser.addOption("Reverse Balance", new ReverseBalance(drivetrain));
    autoChooser.addOption("High  Cube", new HighCube(drivetrain, pivot, arm, manipulator));
    autoChooser.addOption("High Cube Balance", new HighCubeBalance(drivetrain, pivot, arm, manipulator));
    autoChooser.addOption("Right High Cube Mobility", new HighCubeMobility(drivetrain, pivot, arm, manipulator));
    autoChooser.addOption("Left High Cube Mobility", new HighCubeMobilityReverse(drivetrain, pivot, arm, manipulator));
    autoChooser.addOption("High Cube Mobility Balance", new HighCubeMobilityBalance(drivetrain, pivot, arm, manipulator));
    Dashboard.putSendable("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    // Driver button bindings

    // Operator button bindigns
    JoystickButton openPincher = new JoystickButton(operator, 7);
    JoystickButton closePincher = new JoystickButton(operator, 5);
    JoystickButton wristUp = new JoystickButton(operator, 6);
    JoystickButton wristDown = new JoystickButton(operator, 8);
    JoystickButton requestCone = new JoystickButton(operator, 3);
    JoystickButton requestCube = new JoystickButton(operator, 2);
    JoystickButton redLights = new JoystickButton(operator, 14);
    JoystickButton retractArm = new JoystickButton(operator, 15);
    // JoystickButton homePivot = new JoystickButton(operator, 9);
    JoystickButton homeArm = new JoystickButton(operator, 10);
    


    openPincher.onTrue(new InstantCommand(() -> manipulator.openPincher()));
    closePincher.onTrue(new InstantCommand(() -> manipulator.closePincher()));
    wristUp.onTrue(new InstantCommand(() -> manipulator.wristUp()));
    wristDown.onTrue(new InstantCommand(() -> manipulator.wristDown()));
    requestCone.onTrue(new InstantCommand(() -> manipulator.requestCone()));
    requestCube.onTrue(new InstantCommand(() -> manipulator.requestCube()));
    redLights.whileTrue(new RunCommand(() -> manipulator.updateRedPattern()));
    redLights.onFalse(new InstantCommand(() -> manipulator.requestCone()));
    retractArm.onTrue(new AlternateHomeArm(pivot, arm));
    // homePivot.onTrue(new HomePivot(pivot, arm, manipulator));
    homeArm.onTrue(new HomeArm(pivot, arm));

    new POVButton(operator, 270).whileTrue(new SetArmPosition(1, arm));
    new POVButton(operator, 0).whileTrue(new SetPivotPosition(0, pivot));
    new POVButton(operator, 90).whileTrue(new ParallelCommandGroup(new SetPivotPosition(0, pivot), new WaitUntilCommand(() -> pivot.getPivotPosition() > -20).andThen(new SetArmPosition(85, arm))));
    new POVButton(operator, 180).whileTrue(new ParallelCommandGroup(new SetArmPosition(1, arm), new WaitUntilCommand(() -> arm.getArmPosition() < 20).andThen(new SetPivotPosition(-70, pivot))));
    new JoystickButton(operator, 1).whileTrue(new SetArmPosition(20, arm));
    new JoystickButton(operator, 4).whileTrue(new SetArmPosition(80, arm));
    new JoystickButton(driver, 5).whileTrue(new InstantCommand(() -> drivetrain.resetGyro()));
    new JoystickButton(driver, 6).whileTrue(new TurnDegrees(40, drivetrain).andThen(new SetPivotPosition(-56, pivot)));
    
    
    // Dashboard.putSendable("Home Pivot", new HomePivot(pivot, arm, manipulator));
    Dashboard.putSendable("Home Arm", new HomeArm(pivot, arm));
    Dashboard.putSendable("Reset Pivot", new InstantCommand(() -> pivot.resetPivotEncoder(84)).withName("Reset Pivot"));
    Dashboard.putSendable("ALTERNATE ARM", new AlternateHomeArm(pivot, arm));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
