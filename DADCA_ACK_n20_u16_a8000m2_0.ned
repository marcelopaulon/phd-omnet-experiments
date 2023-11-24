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
        @display("i=misc/sensor;p=127.91,7845.27");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1100.79,7932.96");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1600.06,7968.28");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2023.38,7815.39");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2722.66,7876.28");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3097.15,7980.94");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4217.35,7792.55");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4721.62,7859.29");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5292.97,7958.01");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5860.58,7922.62");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=788.39,7723.52");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2397.39,7641.17");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2852.24,7529.55");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3501.50,7742.29");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5429.84,7590.28");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=38.56,7350.24");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1242.18,7325.85");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2324.63,7267.50");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3224.41,7475.22");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4235.00,7335.85");
    }

}

