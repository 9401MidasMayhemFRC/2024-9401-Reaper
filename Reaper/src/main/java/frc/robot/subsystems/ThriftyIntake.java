package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ThriftyNova;

public class ThriftyIntake extends SubsystemBase {
    private final ThriftyNova m_motor = new ThriftyNova(1);

    public ThriftyIntake() {

    }

    public Command runCmd(double percent) {
        return new InstantCommand(() -> run(percent));
    }

    public Command stopCmd() {
        return new InstantCommand(() -> stop());
    }

    public void run(double percent) {
        m_motor.setPercentOutput(-1.0*percent);
    }

    public void stop() {
        m_motor.setPercentOutput(0.0);
    }
}
