package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.IDMap;

public class Manipulator extends SubsystemBase {
    private final Compressor compressor;
    private final DoubleSolenoid conicalPincher, wrist;
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    public Manipulator() {
        compressor = new Compressor(PneumaticsModuleType.REVPH);
        compressor.enableAnalog(95, 115);

        conicalPincher = new DoubleSolenoid(PneumaticsModuleType.REVPH, IDMap.Pneumatics.openPincher.port, IDMap.Pneumatics.closePincher.port);
        conicalPincher.set(DoubleSolenoid.Value.kReverse);

        wrist = new DoubleSolenoid(PneumaticsModuleType.REVPH, IDMap.Pneumatics.wristUp.port, IDMap.Pneumatics.wristDown.port);
        wrist.set(DoubleSolenoid.Value.kForward);

        led = new AddressableLED(IDMap.DIO.LEDs.port);
        buffer = new AddressableLEDBuffer(Constants.Manipulator.LEDLength);
        led.setLength(buffer.getLength());
        requestCone();
        led.start();
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

    // LED Methods
    public void requestCone() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 255, 128, 0);
        }
        led.setData(buffer);
    }

    public void requestCube() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 255, 0, 255);
        }
        led.setData(buffer);
    }

    public void clearLEDs() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 0, 0);
        }
        led.setData(buffer);
    }
}
