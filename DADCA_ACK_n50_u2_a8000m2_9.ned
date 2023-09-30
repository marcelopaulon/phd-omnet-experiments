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

network DADCA_ACK_n50_u2_a8000m2_9
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
        @display("i=misc/sensor;p=960.97,7928.56");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1314.59,7972.00");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1729.11,7757.44");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2492.23,7847.27");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4061.01,7991.11");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5258.16,7918.16");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5766.02,7824.54");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=63.92,7672.61");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=650.59,7587.03");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1129.78,7599.87");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2175.84,7511.24");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3070.80,7603.79");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3491.73,7690.89");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4354.15,7732.42");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4886.90,7579.39");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5527.59,7562.74");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1708.83,7379.50");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2510.54,7308.72");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3463.59,7260.37");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3941.21,7495.98");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=567.43,7228.61");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1433.28,7131.91");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3046.61,7199.95");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3801.80,7146.58");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4322.46,7167.45");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5409.22,7140.64");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5841.34,7193.70");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=211.22,6963.03");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1026.53,6982.96");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1810.09,6929.39");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2462.56,6941.34");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3490.54,6836.01");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4819.87,6953.11");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=745.42,6560.08");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1255.48,6690.17");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2067.03,6561.15");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2480.50,6589.75");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2974.71,6594.85");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4195.13,6733.81");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5368.84,6641.52");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5951.29,6761.33");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=42.63,6431.78");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1572.87,6467.81");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3563.93,6450.27");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4173.16,6363.13");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4739.85,6374.33");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5095.91,6382.27");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5853.39,6388.83");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=599.33,6116.54");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1313.67,6197.72");
    }

}

