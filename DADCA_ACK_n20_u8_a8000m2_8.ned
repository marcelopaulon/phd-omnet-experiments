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

network DADCA_ACK_n20_u8_a8000m2_8
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
        @display("i=misc/sensor;p=2671.17,7926.15");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3370.89,7852.79");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3738.52,7922.40");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4252.28,7980.56");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4963.78,7966.06");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2188.22,7709.66");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3012.30,7634.95");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3901.79,7594.17");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4265.70,7629.96");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4730.94,7522.40");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5332.33,7684.84");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1812.25,7315.30");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2609.26,7283.09");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3445.98,7235.91");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4264.40,7221.32");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5146.55,7084.19");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5864.65,7085.54");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2028.90,6926.43");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2555.25,6875.76");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3163.96,6869.11");
    }

}

