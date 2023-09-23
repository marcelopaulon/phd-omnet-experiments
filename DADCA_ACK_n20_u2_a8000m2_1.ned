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

network DADCA_ACK_n20_u2_a8000m2_1
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
        @display("i=misc/sensor;p=1211.30,7866.08");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1689.49,7917.46");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2063.48,7755.84");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3333.56,7988.19");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4055.89,7987.21");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4469.48,7957.32");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5001.97,7775.08");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5621.84,7943.11");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1497.41,7581.80");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2460.99,7721.99");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2953.34,7625.77");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3895.04,7659.42");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4408.58,7549.69");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=916.01,7337.55");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1927.03,7306.28");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2305.45,7394.32");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3604.19,7422.49");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5242.60,7380.76");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1648.49,7068.07");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2253.20,7017.41");
    }

}

