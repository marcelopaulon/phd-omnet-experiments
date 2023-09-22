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

network DADCA_ACK_n20_a8000m2_1
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
        @display("i=misc/sensor;p=134.79,7908.15");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=739.68,7752.77");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2970.17,7924.10");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3651.72,7883.25");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1616.90,7699.63");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2144.88,7722.17");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2650.90,7635.17");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3457.11,7535.92");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4206.69,7733.84");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5157.21,7672.35");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5708.41,7491.17");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=214.52,7415.34");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2430.15,7231.47");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4519.78,7369.79");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=821.57,7106.84");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1472.10,7122.58");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2052.37,7166.51");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2753.96,7036.20");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3169.38,7188.00");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3608.55,7076.87");
    }

}

