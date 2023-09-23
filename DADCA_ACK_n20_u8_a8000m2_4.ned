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

network DADCA_ACK_n20_u8_a8000m2_4
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
        @display("i=misc/sensor;p=363.45,7973.72");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1483.72,7951.60");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2137.30,7853.17");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3287.58,7944.52");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3744.11,7946.88");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5954.26,7989.16");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=529.96,7588.61");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=918.52,7541.45");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1722.65,7664.80");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2253.25,7513.66");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2669.80,7732.05");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3174.36,7513.95");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3917.69,7552.50");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4301.22,7581.41");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4790.01,7750.52");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5328.37,7710.55");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5847.46,7549.88");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=114.34,7450.49");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1340.47,7421.71");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2896.23,7284.46");
    }

}

