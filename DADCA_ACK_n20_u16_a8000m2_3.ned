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

network DADCA_ACK_n20_u16_a8000m2_3
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
        @display("i=misc/sensor;p=421.20,7977.85");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1037.20,7973.47");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2244.79,7765.16");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3123.10,7929.19");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3557.31,7983.11");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=214.97,7630.95");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1451.70,7535.87");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3631.08,7619.45");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4339.37,7611.68");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5143.09,7746.33");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5700.88,7523.36");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=971.18,7421.08");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2017.57,7493.40");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2832.16,7471.51");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3378.78,7332.23");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4591.40,7333.02");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=193.58,7203.72");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=617.82,7250.10");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1616.13,7011.47");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1955.81,7130.74");
    }

}

