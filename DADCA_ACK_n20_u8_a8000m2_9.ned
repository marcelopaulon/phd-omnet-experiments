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

network DADCA_ACK_n20_u8_a8000m2_9
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
        @display("i=misc/sensor;p=929.82,7798.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1656.91,7941.94");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2276.77,7859.52");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3421.47,7855.32");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4247.82,7845.51");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4627.25,7885.86");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5018.06,7881.42");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5693.23,7788.91");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=285.33,7718.68");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1535.16,7570.19");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3745.10,7696.58");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4429.77,7541.67");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=688.57,7404.88");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1164.96,7379.61");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2037.41,7268.16");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2641.52,7382.39");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3081.24,7297.94");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4921.40,7412.08");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=62.34,7095.09");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1754.56,7053.94");
    }

}

