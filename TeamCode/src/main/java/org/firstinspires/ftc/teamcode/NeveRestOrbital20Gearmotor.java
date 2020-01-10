package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.configuration.DistributorInfo;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.ExpansionHubPIDFPositionParams;
import com.qualcomm.robotcore.hardware.configuration.annotations.ExpansionHubPIDFVelocityParams;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

/**
 * Created by Sean Cardosi on 2020-01-04.
 */
@MotorType(ticksPerRev = 537.6, gearing = 19.2, maxRPM = 340, orientation = Rotation.CW)
@DeviceProperties(xmlTag="NeveRestOrbital20Gearmotor", name="NeveRest Orbital 20 Gearmotor", builtIn = false)
@DistributorInfo(distributor="AndyMark", model="am-3637", url="https://www.andymark.com/products/neverest-orbital-20-gearmotor")
@ExpansionHubPIDFVelocityParams(P=0, I=0, F=0)
@ExpansionHubPIDFPositionParams(P=0)
public interface NeveRestOrbital20Gearmotor {}
