package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Amp extends SubsystemBase{
    
    private CANSparkMax m_motor = new CANSparkMax(17, MotorType.kBrushed);

    private double m_speed;

    public Amp(){

        m_motor.restoreFactoryDefaults();
        m_motor.enableVoltageCompensation(12.0);
        m_motor.setInverted(true);
        m_motor.setIdleMode(IdleMode.kCoast);
        m_motor.burnFlash();

    }

    /**
     * Takes Volts as parameter and then uses if, else if, else to limit the amount of voltage used to prevent the motor from damage. Use ampStop() to stop the motor instead of setting volts to 0.0 it will default to 0.1. Make sure to invert the motor the correct way.
     * @param volts
     */
    public void setSpeed(double speed){
        m_speed = speed;
        m_motor.set(speed);
       }

    public void stop(){
        m_motor.stopMotor();
    }

    public double getCurrent(){
        return m_motor.getOutputCurrent();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Amp Motor Voltage",m_speed);
        SmartDashboard.putNumber("Amp Current", getCurrent());
    }

}
