package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.hal.sim.I2CSim;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.RobotMap;

import java.nio.ByteBuffer;

public class Shooter {
    private static WPI_TalonSRX motorA;
    private static WPI_TalonSRX motorB;


    private static I2C multiplexer = new I2C(I2C.Port.kOnboard, 0x44);

    public static void init() {
        motorA = new WPI_TalonSRX(RobotMap.SHOOTER_A);
        motorB = new WPI_TalonSRX(RobotMap.SHOOTER_B);
        multiplexer.write(2, 2);
        multiplexer.read(2, 2, ByteBuffer.allocateDirect(2));
    }

    public static void loop() {

    }

    public static void setSpeed(double speed) {
        motorA.set(speed);
        motorB.set(speed);
    }
}