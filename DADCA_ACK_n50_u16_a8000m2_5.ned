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

network DADCA_ACK_n50_u16_a8000m2_5
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
        @display("i=misc/sensor;p=1234.48,7991.72");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2449.72,7869.93");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2924.86,7947.24");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3483.73,7991.97");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3812.72,7796.29");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4615.49,7949.49");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5294.02,7994.16");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5791.54,7788.76");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1671.64,7620.30");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2019.87,7720.20");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3328.44,7624.90");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4219.52,7735.73");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=358.57,7446.09");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=860.00,7463.46");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1423.16,7332.30");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2100.17,7295.10");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2554.30,7305.19");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2933.79,7384.63");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3898.46,7361.54");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4572.82,7448.21");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4966.85,7482.55");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5324.55,7485.84");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5915.24,7259.51");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=328.44,7059.17");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3500.56,7148.18");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4303.46,7219.92");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5493.02,7044.87");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1120.09,6989.38");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1535.38,6854.66");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1921.48,6893.48");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2322.52,6926.41");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2677.52,6801.02");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3054.54,6974.42");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3740.42,6868.80");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5157.58,6943.22");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5857.13,6856.05");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=84.73,6551.11");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=525.01,6523.60");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3239.74,6516.11");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4267.79,6605.51");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4839.84,6761.78");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1032.55,6510.77");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1384.71,6476.57");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1820.72,6419.21");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2383.90,6437.94");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3777.36,6482.34");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4803.99,6344.45");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5149.61,6409.59");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5623.10,6440.08");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5996.49,6390.00");
    }

}

