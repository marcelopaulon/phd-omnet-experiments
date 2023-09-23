package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n20_u8_a8000m2_1
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor;p=903.04,7976.65");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1484.35,7759.17");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1971.56,7853.18");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2618.75,7885.46");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3253.38,7877.96");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4595.09,7767.30");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2873.68,7531.26");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3715.75,7508.27");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5304.18,7593.32");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=671.11,7453.34");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=999.40,7327.93");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1696.50,7332.31");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2257.69,7289.70");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4341.48,7501.19");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4972.89,7409.21");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=207.13,7111.86");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2637.19,7114.06");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3341.22,7243.26");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4043.21,7208.79");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4598.55,7048.63");
    }

}

