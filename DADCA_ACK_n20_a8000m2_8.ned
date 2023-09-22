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

network DADCA_ACK_n20_a8000m2_8
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
        @display("i=misc/sensor;p=265.24,7859.92");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=615.58,7954.81");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1158.92,7786.29");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1772.73,7861.65");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2202.99,7874.98");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4347.23,7838.78");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5254.37,7843.51");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=748.43,7532.78");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3047.60,7632.29");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3742.29,7552.36");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4803.17,7680.94");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5884.43,7533.33");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=127.11,7329.41");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1282.71,7304.61");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2071.04,7310.56");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2497.94,7303.25");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3254.50,7314.32");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4414.18,7406.23");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4998.66,7267.92");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3885.56,7047.77");
    }

}

