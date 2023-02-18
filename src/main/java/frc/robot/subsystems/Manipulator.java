package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase {
    private final DoubleSolenoid conicalPincher;
    private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

    public Manipulator() {
        conicalPincher = new DoubleSolenoid(PneumaticsModuleType.REVPH, 7, 14);
        compressor.enableAnalog(95, 115);
    }

    public void openPincher() {
        conicalPincher.set(DoubleSolenoid.Value.kForward);
    }

    public void closePincher() {
        conicalPincher.set(DoubleSolenoid.Value.kReverse);
    }
}
