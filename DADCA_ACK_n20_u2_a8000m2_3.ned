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

network DADCA_ACK_n20_u2_a8000m2_3
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
        @display("i=misc/sensor;p=1163.97,7999.13");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1828.85,7901.38");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2474.32,7944.34");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2967.86,7789.77");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3744.21,7907.53");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4299.08,7756.18");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=647.49,7633.23");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1546.74,7662.05");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3418.97,7687.38");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5055.40,7570.66");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5544.35,7670.11");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5993.32,7511.60");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1076.39,7448.35");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1855.69,7291.99");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2218.95,7359.82");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2841.95,7380.78");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3495.48,7288.36");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3896.73,7432.90");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4509.65,7313.94");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=353.32,7204.41");
    }

}

