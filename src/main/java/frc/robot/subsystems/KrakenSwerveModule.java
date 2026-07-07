package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.MotorConstants;

public class KrakenSwerveModule extends SwerveModuleBase {

    private static MotorType[] motorTypes = {MotorType.TALON, MotorType.TALON};

    private TalonFXConfiguration driveConfig, turnConfig;

    public KrakenSwerveModule(int[] ids, int rotationEncoderID, String name) throws Exception {
        super(ids, motorTypes, rotationEncoderID, name);
        configureDrive();
        configureTurn();
    }

    public void configureDrive() {

        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.CurrentLimits.StatorCurrentLimit =
                MotorConstants.driveStatorCurrentLimit.abs(Amps);
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.CurrentLimits.SupplyCurrentLimit =
                MotorConstants.driveSupplyCurrentLimit.abs(Amps);

        driveConfig.Slot0.kP = MotorConstants.driveP;
        driveConfig.Slot0.kI = MotorConstants.driveI;
        driveConfig.Slot0.kD = MotorConstants.driveD;
        driveConfig.Slot0.kV = MotorConstants.driveKV;

        try {
            configure(driveConfig, "Driving Motor");
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void configureTurn() {

        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;

        turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turnConfig.CurrentLimits.StatorCurrentLimit =
                MotorConstants.turnStatorCurrentLimit.abs(Amps);
        turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turnConfig.CurrentLimits.SupplyCurrentLimit =
                MotorConstants.turnSupplyCurrentLimit.abs(Amps);

        turnConfig.Slot0.kP = MotorConstants.turnP;
        turnConfig.Slot0.kI = MotorConstants.turnI;
        turnConfig.Slot0.kD = MotorConstants.turnD;
        turnConfig.Slot0.kV = MotorConstants.turnKV;

        try {
            configure(turnConfig, "Turning Motor");
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

}
