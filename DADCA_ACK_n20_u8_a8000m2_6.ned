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

network DADCA_ACK_n20_u8_a8000m2_6
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
        @display("i=misc/sensor;p=279.74,7813.27");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=689.72,7856.67");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1262.66,7973.92");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2027.80,7917.36");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2658.40,7761.60");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3140.63,7782.58");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3759.36,7897.26");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4578.91,7765.75");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2255.35,7640.03");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3526.51,7562.79");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5111.57,7519.02");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5928.79,7735.89");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=266.07,7309.43");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=776.78,7504.56");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1259.93,7444.44");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1621.34,7366.34");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2477.54,7363.74");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2862.32,7392.47");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4510.05,7366.31");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5560.39,7481.20");
    }

}

