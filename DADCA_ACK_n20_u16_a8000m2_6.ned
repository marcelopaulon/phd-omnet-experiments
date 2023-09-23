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

network DADCA_ACK_n20_u16_a8000m2_6
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
        @display("i=misc/sensor;p=1266.71,7987.99");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2101.13,7810.38");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4702.33,7861.81");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=17.73,7719.43");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=428.74,7668.86");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=984.58,7597.52");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1338.27,7509.61");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1778.97,7570.09");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2529.52,7727.77");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2956.29,7688.13");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4072.82,7559.31");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2840.02,7316.60");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3552.09,7430.27");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4418.42,7299.66");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4844.75,7414.00");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=266.36,7014.19");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=901.28,7096.54");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1417.42,7066.29");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1829.26,7073.61");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2267.20,7215.75");
    }

}

