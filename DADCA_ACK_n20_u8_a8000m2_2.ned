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

network DADCA_ACK_n20_u8_a8000m2_2
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
        @display("i=misc/sensor;p=590.97,7970.09");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2005.97,7978.99");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3702.71,7804.85");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4702.62,7782.05");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5109.77,7925.21");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5463.56,7758.11");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=599.94,7601.79");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1017.28,7647.07");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1511.34,7535.24");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2681.61,7677.17");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3145.36,7548.62");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4215.09,7649.57");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5916.32,7703.77");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2007.44,7405.06");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2474.33,7356.29");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4155.21,7277.49");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=340.67,7019.35");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=627.73,7248.94");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=951.39,7068.34");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3184.01,7022.17");
    }

}

