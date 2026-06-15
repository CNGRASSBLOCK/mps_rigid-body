package org.polaris2023.msp_rigid_body;

import net.neoforged.fml.common.Mod;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@Mod(MpsRigidBodyMod.MOD_ID)
public final class MpsRigidBodyMod {
    public static final String MOD_ID = "mps_rigid_body";
    private static final Logger LOGGER = LoggerFactory.getLogger(MpsRigidBodyMod.class);

    public MpsRigidBodyMod() {
        LOGGER.info("mps_rigid_body mod loaded");
    }
}
