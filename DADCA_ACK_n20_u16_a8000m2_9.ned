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

network DADCA_ACK_n20_u16_a8000m2_9
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
        @display("i=misc/sensor;p=426.29,7905.34");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1845.33,7778.30");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3026.64,7942.38");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3408.90,7824.09");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4246.22,7944.49");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4686.17,7818.74");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5351.28,7954.06");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5940.26,7817.86");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=836.37,7550.31");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1388.15,7595.01");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2423.41,7672.13");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3965.50,7648.14");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5250.83,7569.96");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=194.99,7387.24");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1929.53,7291.91");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2648.13,7392.29");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3482.85,7409.15");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4136.03,7336.10");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4629.99,7304.80");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5813.39,7463.73");
    }

}

