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
        @display("i=misc/sensor;p=418.75,7763.81");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=990.51,7842.99");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2346.41,7961.52");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3166.70,7942.98");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4515.66,7919.65");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4883.15,7933.69");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5253.79,7861.22");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5724.28,7779.85");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1316.44,7555.18");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1713.29,7681.52");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3843.74,7743.76");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4821.58,7578.97");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=378.62,7347.87");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=732.86,7330.96");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1805.03,7275.01");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2174.93,7464.27");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2854.61,7449.63");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3315.54,7356.66");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3708.70,7273.81");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4342.87,7448.93");
    }

}

