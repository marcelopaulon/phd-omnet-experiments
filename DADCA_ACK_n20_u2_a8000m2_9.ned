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

network DADCA_ACK_n20_u2_a8000m2_9
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
        @display("i=misc/sensor;p=3.74,7829.57");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=602.09,7851.88");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1579.77,7845.81");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3941.13,7816.71");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4709.35,7770.96");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5124.21,7753.59");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5942.86,7950.34");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2250.66,7660.06");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2882.94,7624.48");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3404.97,7735.48");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5554.66,7737.22");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5866.21,7556.49");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1005.60,7476.43");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1903.92,7493.80");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3394.82,7341.77");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3770.97,7450.39");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4188.15,7460.52");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4520.82,7284.68");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5406.06,7383.66");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=80.04,7159.83");
    }

}

