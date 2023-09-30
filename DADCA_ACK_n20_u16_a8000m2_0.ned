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

network DADCA_ACK_n20_u16_a8000m2_0
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
        @display("i=misc/sensor;p=344.63,7890.33");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1012.41,7769.01");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2906.43,7797.88");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=190.09,7558.11");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=744.59,7516.16");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1515.92,7648.42");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2129.49,7695.37");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3450.13,7663.18");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4211.69,7745.51");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4602.51,7636.57");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5010.50,7713.99");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1319.92,7314.01");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2240.96,7263.12");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2886.08,7311.46");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3819.67,7408.48");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5598.73,7427.80");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=476.47,7139.68");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=905.23,7088.07");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1791.30,7111.09");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3337.78,7020.74");
    }

}

