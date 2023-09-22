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

network DADCA_ACK_n20_a8000m2_6
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
        @display("i=misc/sensor;p=1307.46,7822.59");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3060.34,7950.99");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3818.18,7885.58");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4706.91,7997.37");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5523.86,7974.48");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=135.22,7740.27");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1821.00,7665.49");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2751.52,7730.07");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3134.04,7573.89");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3471.77,7668.28");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4147.30,7693.21");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5135.82,7720.36");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5836.34,7594.51");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=447.17,7393.51");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1814.05,7287.01");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2568.56,7320.84");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5402.02,7357.20");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=908.94,7249.95");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1382.40,7195.70");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2215.01,7217.99");
    }

}

