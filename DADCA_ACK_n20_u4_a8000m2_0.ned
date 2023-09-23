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

network DADCA_ACK_n20_u4_a8000m2_0
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
        @display("i=misc/sensor;p=561.28,7755.85");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1071.86,7775.12");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2289.11,7990.58");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3945.86,7923.84");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4446.45,7826.59");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5774.68,7992.72");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1998.29,7748.26");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2374.71,7588.13");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3288.62,7619.86");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3860.98,7517.34");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4852.86,7711.77");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5322.11,7734.16");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=542.14,7267.20");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1107.57,7332.03");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1466.64,7495.70");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2068.47,7369.99");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2707.10,7353.03");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4347.78,7471.93");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4795.49,7260.19");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5461.39,7376.84");
    }

}

