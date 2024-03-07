package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OIConstants;

public class LEDs extends SubsystemBase {
    private final AddressableLED m_leds;
    private final AddressableLEDBuffer m_ledBuffer;

    private Color curColor = OIConstants.kLedOrange;

    public LEDs(int pwmPort, int ledLength) {
        m_leds = new AddressableLED(pwmPort);
        m_ledBuffer = new AddressableLEDBuffer(ledLength);

        m_leds.setLength(ledLength);
        m_leds.start();
    }

    @Override
    public void periodic() {
        m_leds.setData(m_ledBuffer);

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setLED(i, curColor);
         }
    }

    public Command setColor(Color color) {
        return run(
            () -> curColor = color
        );
    }

    public void enable() {
        m_leds.start();
    }

    public void disable() {
        m_leds.stop();
    }
}