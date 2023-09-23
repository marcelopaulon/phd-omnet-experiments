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
        @display("i=misc/sensor;p=1441.58,7890.61");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1832.34,7831.16");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2524.99,7914.03");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3126.61,7899.97");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4047.85,7824.80");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4441.04,7978.63");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=815.44,7737.50");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2097.50,7540.12");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2743.99,7611.76");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3706.98,7566.99");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4610.02,7539.07");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5064.85,7688.65");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5532.39,7701.12");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=354.91,7316.23");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=825.91,7314.23");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1185.18,7348.31");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1723.90,7287.59");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5270.52,7361.39");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2276.63,7100.01");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2830.47,7113.97");
    }

}

