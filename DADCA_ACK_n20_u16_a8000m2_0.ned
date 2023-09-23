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
        @display("i=misc/sensor;p=788.78,7970.46");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2562.44,7802.56");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3396.85,7843.53");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4688.88,7894.69");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5315.97,7865.24");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5779.70,7830.16");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=329.62,7585.99");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1186.20,7700.14");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1733.29,7675.38");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2952.12,7674.80");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5013.46,7555.56");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5557.87,7556.11");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=806.15,7486.77");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1655.92,7308.95");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2310.29,7412.77");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3266.06,7281.63");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3904.48,7456.55");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4749.17,7307.78");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=43.52,7122.49");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=937.86,7084.90");
    }

}

