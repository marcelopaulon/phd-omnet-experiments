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

network DADCA_ACK_n20_u8_a8000m2_2
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
        @display("i=misc/sensor;p=65.37,7883.91");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1020.03,7974.20");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2031.66,7994.08");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2853.95,7857.72");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3570.06,7994.18");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3893.04,7808.57");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4450.53,7977.65");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4879.04,7801.13");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5380.07,7788.28");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5807.18,7973.17");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=662.11,7681.24");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1707.69,7653.37");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2076.53,7557.92");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2557.22,7577.23");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3220.98,7701.30");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4339.69,7632.87");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=387.88,7386.12");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1246.47,7431.87");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2924.92,7360.96");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3349.89,7361.80");
    }

}

