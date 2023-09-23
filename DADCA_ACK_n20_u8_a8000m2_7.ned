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

network DADCA_ACK_n20_u8_a8000m2_7
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
        @display("i=misc/sensor;p=564.99,7879.22");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=932.37,7790.59");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1508.53,7759.55");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2497.54,7864.74");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3993.93,7898.91");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5860.94,7755.19");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1862.97,7654.72");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2689.95,7551.61");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3253.08,7628.46");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4579.10,7590.11");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5487.79,7668.32");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=170.22,7501.23");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=478.79,7273.46");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1210.25,7478.66");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2389.81,7271.24");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2988.18,7267.26");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4970.25,7380.66");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5958.28,7407.36");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=70.55,7057.31");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1475.13,7199.03");
    }

}

