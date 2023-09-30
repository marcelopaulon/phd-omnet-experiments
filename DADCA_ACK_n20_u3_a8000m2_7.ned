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

network DADCA_ACK_n20_u3_a8000m2_7
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
        @display("i=misc/sensor;p=71.13,7925.91");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=400.87,7759.86");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1120.12,7842.89");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3322.61,7928.79");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3732.56,7841.63");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4215.02,7808.26");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5044.89,7879.70");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=743.82,7672.63");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1592.73,7663.20");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2011.81,7752.07");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3180.02,7527.30");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4634.77,7685.92");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5362.23,7518.99");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=87.78,7447.52");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=745.27,7270.07");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2037.46,7315.65");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2557.29,7431.58");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3502.66,7372.84");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4012.50,7442.04");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4692.57,7330.70");
    }

}

