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

network DADCA_ACK_n20_u4_a8000m2_0
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
        @display("i=misc/sensor;p=338.21,7828.59");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1321.51,7939.26");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1871.47,7966.44");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3068.62,7843.42");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4685.42,7944.29");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=942.78,7700.31");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1523.23,7630.10");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2732.42,7698.90");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3389.16,7614.71");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4155.44,7570.74");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5031.02,7679.31");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=696.08,7428.27");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1145.89,7385.89");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2983.57,7369.16");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4554.00,7396.72");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=381.17,7029.92");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1722.43,7253.08");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2076.97,7090.77");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2679.18,7043.61");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3865.28,7234.83");
    }

}

