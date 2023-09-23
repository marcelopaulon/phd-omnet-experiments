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

network DADCA_ACK_n20_u4_a8000m2_0
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
        @display("i=misc/sensor;p=1449.84,7787.04");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2306.13,7769.33");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2775.28,7992.42");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3062.48,7783.09");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3514.18,7845.64");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3875.91,7918.72");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4227.98,7800.93");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5067.60,7835.35");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5508.43,7892.94");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=559.74,7559.96");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1077.29,7739.45");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1959.94,7567.34");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4018.10,7513.65");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4690.11,7660.58");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5852.57,7671.44");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=139.77,7302.76");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1568.84,7288.55");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2575.52,7291.36");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3542.83,7365.41");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5178.92,7301.72");
    }

}

