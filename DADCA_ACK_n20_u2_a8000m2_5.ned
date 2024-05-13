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

network DADCA_ACK_n20_u2_a8000m2_5
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
        @display("i=misc/sensor;p=708.91,7962.51");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1673.53,7918.32");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2256.45,7847.04");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4968.55,7835.03");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5710.89,7965.11");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=402.88,7581.19");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=770.68,7522.33");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1143.69,7561.56");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4452.29,7721.75");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=51.12,7384.35");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1786.90,7340.21");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2583.92,7275.23");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3561.82,7390.73");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4068.77,7417.08");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4620.11,7295.09");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5426.50,7359.74");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=495.65,7127.41");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1054.81,7210.85");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3135.70,7227.33");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3875.06,7073.28");
    }

}

