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

network DADCA_ACK_n20_u3_a8000m2_6
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
        @display("i=misc/sensor;p=1019.61,7799.23");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1620.51,7794.55");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3164.78,7886.77");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4227.19,7848.00");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4987.15,7777.64");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2362.90,7667.71");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2720.55,7728.42");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3047.86,7547.15");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3706.50,7664.33");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5786.51,7523.56");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=912.34,7377.06");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1509.75,7404.82");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1833.47,7266.46");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2444.28,7290.48");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3481.56,7289.46");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4460.59,7353.06");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5375.00,7448.17");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2810.72,7183.46");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4141.43,7158.34");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5053.55,7254.80");
    }

}

