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

network DADCA_ACK_n20_u8_a8000m2_8
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
        @display("i=misc/sensor;p=32.73,7896.71");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=619.66,7918.07");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1763.55,7769.17");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2684.81,7970.80");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4770.56,7873.00");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5462.90,7807.86");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=582.89,7541.49");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=958.80,7714.51");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2084.35,7562.07");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2465.47,7668.80");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2834.40,7612.03");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3209.32,7687.17");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3585.33,7669.32");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4167.91,7510.35");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4518.80,7508.60");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4996.51,7565.50");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5922.17,7600.42");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1481.69,7410.30");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3030.32,7264.65");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5515.48,7289.58");
    }

}

