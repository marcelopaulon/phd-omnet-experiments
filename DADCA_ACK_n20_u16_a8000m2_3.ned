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

network DADCA_ACK_n20_u16_a8000m2_3
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
        @display("i=misc/sensor;p=126.85,7988.32");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2925.38,7961.69");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3480.57,7813.44");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3995.34,7912.04");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4897.33,7919.03");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=367.02,7732.00");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1149.35,7556.86");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1678.08,7547.53");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2316.69,7664.73");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5336.75,7646.81");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5943.62,7681.35");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=154.94,7420.74");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=576.58,7364.45");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2900.58,7486.21");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3806.47,7331.44");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4207.74,7273.91");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4550.37,7346.44");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5901.21,7306.78");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=34.99,7054.85");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1206.78,7064.80");
    }

}

