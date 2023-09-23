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

network DADCA_ACK_n50_u4_a8000m2_4
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
        @display("i=misc/sensor;p=281.48,7776.58");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=923.96,7828.09");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1608.06,7966.65");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2267.60,7862.26");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2914.19,7826.18");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3472.39,7897.98");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3967.18,7938.15");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4391.79,7863.69");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5114.49,7836.37");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5813.25,7802.73");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1370.45,7605.09");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1933.58,7511.56");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5604.04,7510.75");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=866.09,7381.47");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2949.52,7475.23");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3737.32,7444.61");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4348.15,7383.81");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4729.63,7491.68");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=79.07,7144.54");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=435.72,7103.45");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2093.18,7176.80");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2518.38,7252.61");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4780.73,7097.44");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5279.22,7178.98");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5801.46,7076.89");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=859.81,6936.08");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1778.09,6844.89");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3415.92,7000.59");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3857.43,6907.59");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=594.14,6597.39");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1355.50,6707.31");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2555.90,6611.28");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2986.38,6618.20");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3673.79,6591.96");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4510.12,6712.98");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5018.07,6605.79");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=158.81,6461.64");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=845.92,6321.31");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1636.85,6371.00");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2263.27,6411.58");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3318.68,6487.07");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4064.66,6505.18");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5691.51,6469.69");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=64.02,6075.04");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=456.42,6098.48");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1141.40,6067.47");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2136.06,6054.92");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2703.26,6164.41");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3262.06,6021.60");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3861.87,6128.04");
    }

}

