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

network DADCA_ACK_n20_u8_a8000m2_3
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
        @display("i=misc/sensor;p=624.55,7963.23");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1207.55,7886.39");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2885.83,7871.95");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3318.74,7948.90");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4001.67,7875.33");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=99.21,7743.57");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1877.01,7571.60");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2384.90,7505.13");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4373.55,7539.22");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4822.54,7664.78");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5425.75,7714.99");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=399.70,7384.78");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=969.88,7459.52");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1310.42,7280.05");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2827.32,7461.35");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3750.45,7488.83");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5401.86,7361.28");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5963.73,7459.29");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=101.49,7055.18");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=789.71,7132.63");
    }

}

