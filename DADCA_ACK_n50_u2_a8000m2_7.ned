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

network DADCA_ACK_n50_u2_a8000m2_7
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
        @display("i=misc/sensor;p=899.38,7946.16");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2670.18,7916.93");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3197.53,7942.27");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3796.47,7834.50");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4398.34,7828.60");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4936.81,7982.48");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1502.81,7634.01");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1857.69,7721.13");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2172.47,7539.05");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5396.14,7670.11");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=174.86,7448.48");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=545.57,7442.60");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=978.79,7257.95");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2524.20,7463.91");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2978.02,7339.20");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3484.66,7428.39");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4167.58,7333.56");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4624.40,7440.20");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5265.99,7257.83");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=399.91,7111.55");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1481.58,7024.60");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2691.01,7111.01");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3767.83,7045.44");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4760.58,7104.66");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5746.98,7101.91");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=13.48,6814.98");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=744.19,6964.82");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2290.67,6941.81");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3262.41,6953.44");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4244.48,6802.72");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=403.47,6569.61");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=871.58,6589.95");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1793.72,6717.15");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2534.49,6555.53");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3552.35,6563.80");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5174.63,6711.61");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5508.85,6525.01");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1469.31,6482.60");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2098.42,6396.80");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3165.63,6488.42");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3974.28,6424.13");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4684.65,6477.00");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=361.48,6170.91");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1477.24,6088.70");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2895.75,6216.33");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3544.02,6068.07");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4399.76,6216.74");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5268.95,6055.49");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5628.89,6188.14");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=715.33,5845.75");
    }

}

