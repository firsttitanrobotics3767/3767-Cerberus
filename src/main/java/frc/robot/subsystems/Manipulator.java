package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.IDMap;

public class Manipulator extends SubsystemBase {
    private final DoubleSolenoid conicalPincher, wrist;
    private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

    public Manipulator() {
        compressor.enableAnalog(95, 115);
        conicalPincher = new DoubleSolenoid(PneumaticsModuleType.REVPH, IDMap.Pneumatics.openPincher.port, IDMap.Pneumatics.closePincher.port);
        wrist = new DoubleSolenoid(PneumaticsModuleType.REVPH, IDMap.Pneumatics.wristUp.port, IDMap.Pneumatics.wristDown.port);
    }

    public void openPincher() {
        conicalPincher.set(DoubleSolenoid.Value.kForward);
    }

    public void closePincher() {
        conicalPincher.set(DoubleSolenoid.Value.kReverse);
    }

    public void togglePincher() {
        conicalPincher.toggle();
    }

    public void wristUp() {
        wrist.set(DoubleSolenoid.Value.kForward);
    }

    public void wristDown() {
        wrist.set(DoubleSolenoid.Value.kReverse);
    }

    public void toggleWrist() {
        wrist.toggle();
    }
}
