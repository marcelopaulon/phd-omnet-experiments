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

network DADCA_ACK_n20_u16_a8000m2_7
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
        @display("i=misc/sensor;p=1224.36,7916.68");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1886.70,7926.37");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2560.40,7985.28");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2954.62,7941.35");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3484.86,7944.26");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3835.35,7860.91");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4345.98,7814.72");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5184.99,7861.66");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=643.45,7677.49");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2229.44,7678.06");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2718.74,7594.09");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3479.38,7531.53");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4772.41,7720.20");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1434.06,7402.82");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1998.94,7304.90");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3094.13,7496.80");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4002.34,7356.20");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4501.15,7495.88");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4971.50,7362.78");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5467.17,7444.60");
    }

}

