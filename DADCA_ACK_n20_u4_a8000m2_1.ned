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

network DADCA_ACK_n20_u4_a8000m2_1
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
        @display("i=misc/sensor;p=91.63,7861.30");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1034.27,7958.44");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1396.38,7793.28");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2387.40,7855.42");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2796.77,7807.60");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4193.26,7813.23");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5.18,7521.19");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=619.40,7580.00");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3263.01,7634.09");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3857.92,7634.97");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1080.36,7329.83");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2393.92,7486.51");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2874.43,7362.97");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5097.99,7456.15");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5505.10,7403.71");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5963.00,7482.27");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=249.93,7244.21");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=880.13,7025.28");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1943.40,7084.70");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2567.57,7162.57");
    }

}

