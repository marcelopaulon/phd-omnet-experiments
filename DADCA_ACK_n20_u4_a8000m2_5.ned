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

network DADCA_ACK_n20_u4_a8000m2_5
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
        @display("i=misc/sensor;p=37.89,7975.29");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=4275.84,7865.23");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4703.20,7951.96");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5646.09,7850.57");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=487.74,7524.77");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1172.72,7621.53");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1650.80,7682.14");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2154.28,7643.06");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2624.63,7552.67");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2939.21,7751.75");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3562.45,7626.38");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4794.67,7506.90");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5954.27,7534.30");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4088.53,7474.92");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5180.26,7488.06");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=41.34,7128.14");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=457.16,7167.38");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=860.29,7051.86");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1320.42,7115.53");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1701.44,7230.46");
    }

}

