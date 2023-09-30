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
        @display("i=misc/sensor;p=459.60,7791.42");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1660.89,7914.77");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2108.47,7800.65");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2630.11,7908.14");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3201.88,7889.85");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4462.49,7993.65");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=40.53,7744.04");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=968.54,7732.35");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1578.27,7525.84");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4009.44,7589.61");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4542.99,7632.68");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5196.23,7698.07");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5538.09,7529.09");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=536.30,7292.89");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=912.31,7332.17");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2243.43,7339.63");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3070.35,7350.70");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3481.48,7358.52");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=106.25,7025.79");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1309.66,7223.74");
    }

}

