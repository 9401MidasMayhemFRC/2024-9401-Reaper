package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RackPinion;
import frc.robot.utilities.LEDs;

public class IntakeNote extends Command {
    private final Timer m_timer = new Timer();
    private boolean m_noteInRobot = false;
    private boolean m_finished = false;
    private final RackPinion m_rackPinion;
    private final Intake m_intake;
    private final Feeder m_feeder;
    private final Elevator m_elevator;
    private final LEDs m_led;
    private final double m_lowestRackPose = 27.5;
    private final double m_rackPose = m_lowestRackPose + 3.5;

    public IntakeNote(Intake intake, RackPinion rackpinion, Feeder feeder, Elevator elevator, LEDs led) {
        m_rackPinion = rackpinion;
        m_feeder = feeder;
        m_intake = intake;
        m_elevator = elevator;
        m_led = led;

        addRequirements(m_feeder, m_intake, m_rackPinion, m_elevator);

    }

    // Method called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_noteInRobot = false;
        m_finished = false;
        m_rackPinion.runRack();
        m_rackPinion.setPose(m_rackPose);
        m_feeder.setFeedVelo(50.0);
        m_elevator.setPosition(ElevatorConstants.kRestPose);

        m_timer.reset();
        m_timer.start();

    }

    // Method called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double time = m_timer.get();
        if (m_rackPinion.getPosition() >= (m_rackPose - 7.0)) {
            m_intake.run(1.0);
        }

        if (!m_noteInRobot && m_feeder.getProx() && time > 0.250) {
            m_led.setGreen();
            m_noteInRobot = true;
        }
    }

    // Method called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        if (m_noteInRobot == false){
            m_led.setTeamColors();
        }

        m_intake.stop();
        m_feeder.stopFeed();
        m_rackPinion.setPose(5.0);

        if (interrupted == false) {
            // m_led.set();
        }

    }

    // Method that when call returns wither this command has been completed or not.
    @Override
    public boolean isFinished() {
        return m_finished;
    }

}
