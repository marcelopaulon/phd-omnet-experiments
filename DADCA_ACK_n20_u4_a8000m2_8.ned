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

network DADCA_ACK_n20_u4_a8000m2_8
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
        @display("i=misc/sensor;p=501.73,7760.96");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1253.79,7948.17");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3227.41,7901.43");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3888.94,7834.96");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4283.26,7986.10");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4579.11,7791.18");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5750.80,7966.66");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=11.04,7672.57");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=949.77,7674.38");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1591.04,7610.73");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2181.72,7639.91");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4880.95,7562.28");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5244.15,7647.95");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5927.60,7640.43");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=122.99,7271.44");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=633.30,7431.45");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1412.76,7274.47");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3024.01,7491.19");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3594.27,7384.23");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4208.23,7450.54");
    }

}

