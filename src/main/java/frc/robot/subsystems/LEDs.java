package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    private final AddressableLED m_leds;
    private final AddressableLEDBuffer m_ledBuffer;

    public LEDs(int pwmPort, int ledLength) {
        m_leds = new AddressableLED(pwmPort);
        m_ledBuffer = new AddressableLEDBuffer(ledLength);

        m_leds.setLength(ledLength);
        m_leds.start();
    }

    @Override
    public void periodic() {
        m_leds.setData(m_ledBuffer);
    }

    public void setColor(int r, int g, int b) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
         }
    }

    public void enable() {
        m_leds.start();
    }

    public void disable() {
        m_leds.stop();
    }
}