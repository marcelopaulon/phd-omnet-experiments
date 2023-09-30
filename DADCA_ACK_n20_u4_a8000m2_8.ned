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
        @display("i=misc/sensor;p=157.96,7954.54");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=555.35,7899.96");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1219.56,7779.42");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1927.77,7845.32");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2289.17,7942.99");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3570.22,7883.97");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4218.71,7881.45");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5307.84,7880.70");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3051.34,7744.54");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3363.10,7524.77");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4882.33,7733.39");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5672.35,7606.18");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=195.30,7325.73");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=729.45,7371.58");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1687.60,7433.91");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2216.25,7441.27");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2601.51,7295.25");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3015.00,7369.95");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3874.32,7463.77");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4385.56,7471.00");
    }

}

