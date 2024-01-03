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

network DADCA_ACK_n20_u3_a8000m2_0
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
        @display("i=misc/sensor;p=147.86,7932.68");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=640.41,7875.85");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1611.09,7883.32");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2465.20,7885.18");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5274.59,7942.51");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1212.72,7714.15");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2021.84,7539.21");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2863.36,7641.75");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4198.50,7595.09");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4899.67,7723.54");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=353.45,7275.64");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1569.00,7466.14");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3208.21,7401.09");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3559.15,7341.81");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4479.88,7362.66");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5073.24,7382.88");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5482.30,7403.63");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1094.63,7129.61");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1538.07,7114.36");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1957.76,7153.06");
    }

}

