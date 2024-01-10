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
        @display("i=misc/sensor;p=216.79,7900.07");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3696.72,7856.02");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=5874.33,7785.64");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1407.10,7550.68");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2133.51,7712.48");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4042.35,7596.36");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5432.25,7673.87");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=787.75,7501.34");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1795.16,7447.94");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3440.47,7483.73");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4431.29,7279.12");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5996.58,7421.37");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=388.92,7046.26");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1906.44,7066.30");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2263.63,7070.21");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2909.87,7043.51");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3988.27,7214.97");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5121.70,7189.42");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=902.88,6898.43");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1353.08,6990.02");
    }

}

