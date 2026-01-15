package frc.robot.commands.autoalign;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriverBlendController {

    public ChassisSpeeds blend(
        ChassisSpeeds auto,
        ChassisSpeeds driver,
        double autoWeight
    ) {
        double driverWeight = 1.0 - autoWeight;

        return new ChassisSpeeds(
            auto.vxMetersPerSecond * autoWeight +
                driver.vxMetersPerSecond * driverWeight,

            auto.vyMetersPerSecond * autoWeight +
                driver.vyMetersPerSecond * driverWeight,

            auto.omegaRadiansPerSecond * autoWeight +
                driver.omegaRadiansPerSecond * driverWeight
        );
    }
}
