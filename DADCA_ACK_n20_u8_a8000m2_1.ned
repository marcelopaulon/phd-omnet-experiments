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

network DADCA_ACK_n20_u8_a8000m2_1
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
        @display("i=misc/sensor;p=200.17,7769.61");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1159.44,7794.29");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1938.85,7944.32");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3096.21,7932.57");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3682.03,7977.94");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4452.88,7831.06");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5520.85,7854.51");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=533.82,7639.02");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1609.08,7609.55");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2629.94,7671.90");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3399.15,7522.12");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3967.76,7731.22");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4784.82,7697.75");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5177.04,7583.90");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=866.75,7300.94");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2075.33,7486.13");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3096.14,7326.14");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5800.43,7414.66");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=193.39,7194.36");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1208.42,7130.99");
    }

}

