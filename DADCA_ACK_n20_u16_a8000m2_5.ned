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

network DADCA_ACK_n20_u16_a8000m2_5
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
        @display("i=misc/sensor;p=1272.78,7957.69");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1702.57,7883.54");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2060.49,7797.29");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2436.13,7944.69");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3300.85,7819.46");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4815.42,7955.32");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5173.99,7974.40");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=843.28,7567.67");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1216.11,7519.66");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1605.76,7518.38");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2642.77,7536.36");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3659.62,7563.81");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5308.39,7612.03");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=514.25,7384.97");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2271.00,7300.82");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3089.19,7486.61");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5855.30,7482.96");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1208.51,7098.09");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1605.20,7151.92");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3322.41,7058.57");
    }

}

