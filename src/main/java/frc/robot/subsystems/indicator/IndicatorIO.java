package frc.robot.subsystems.indicator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.drivers.led.AddressableLEDPattern;
import frc.robot.drivers.led.patterns.BlinkingPattern;
import frc.robot.drivers.led.patterns.RisingPattern;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.Seconds;

public interface IndicatorIO {
    default Color allianceColor() {
        return switch (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)) {
            case Blue -> Color.kBlue;
            case Red -> Color.kRed;
        };
    }

    void updateInputs(IndicatorIOInputs inputs);

    void setPattern(Patterns pattern);

    void reset();

    enum Patterns {
        NORMAL(null),
        INTAKE(new BlinkingPattern(Color.kRed, 0.2)),
        AFTER_INTAKE(new BlinkingPattern(Color.kGreen, 0.02)),
        PRE_SHOOT(new RisingPattern(Color.kRed, Seconds.of(1))),
        SHOOT(new BlinkingPattern(Color.kYellow, 0.05));

        public final AddressableLEDPattern pattern;

        Patterns(AddressableLEDPattern color) {
            this.pattern = color;
        }
    }

    @AutoLog
    class IndicatorIOInputs {
        public Patterns currentPattern;
    }
}
