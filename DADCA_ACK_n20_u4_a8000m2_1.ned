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

network DADCA_ACK_n20_u4_a8000m2_1
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
        @display("i=misc/sensor;p=1480.63,7819.91");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2045.37,7982.43");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2878.18,7802.90");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3702.95,7919.13");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4166.62,7906.00");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5564.27,7921.58");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=655.45,7671.88");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3340.94,7707.50");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3905.65,7577.16");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4590.01,7510.22");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4959.53,7742.48");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=251.40,7423.46");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=908.34,7349.62");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1327.81,7486.06");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1967.33,7442.39");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2396.11,7461.80");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3319.59,7352.47");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5206.40,7372.18");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5861.90,7323.91");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2694.37,7043.98");
    }

}

