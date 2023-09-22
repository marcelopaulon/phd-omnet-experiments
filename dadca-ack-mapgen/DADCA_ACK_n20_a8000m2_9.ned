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

network DADCA_ACK_n20_a8000m2_9
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
        @display("i=misc/sensor;p=125.33,7962.67");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=499.18,7966.79");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=579.40,7986.66");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=206.28,7925.47");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=276.61,7919.38");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=363.52,7944.18");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=440.40,7924.92");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=644.46,7927.57");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=710.33,7940.95");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=775.79,7953.20");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=874.86,7948.74");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=950.51,7922.21");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=71.72,7909.48");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=355.26,7880.46");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=535.67,7907.82");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=599.45,7878.58");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=799.89,7886.72");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=891.03,7889.70");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=993.45,7876.49");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=36.98,7829.42");
    }

}

