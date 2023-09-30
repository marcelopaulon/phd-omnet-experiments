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

network DADCA_ACK_n20_u16_a8000m2_2
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
        @display("i=misc/sensor;p=411.60,7936.75");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1584.15,7807.80");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2156.14,7947.65");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2714.88,7974.46");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3129.17,7894.80");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3864.99,7932.46");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=249.02,7551.09");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1216.96,7749.88");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2002.30,7525.81");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2509.68,7672.28");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4326.92,7662.82");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4755.30,7593.10");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=628.67,7368.43");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1030.78,7410.22");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3414.16,7422.21");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3905.75,7373.54");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4589.70,7280.55");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5177.14,7307.13");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=265.08,7081.70");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1525.52,7124.43");
    }

}

