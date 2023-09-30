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

network DADCA_ACK_n100_u3_a8000m2_2
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
        @display("i=misc/sensor;p=110.04,7753.49");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=745.62,7754.54");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1695.24,7796.53");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2772.52,7843.31");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3583.26,7874.47");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4119.88,7848.84");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4795.04,7785.61");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5879.93,7974.15");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1254.48,7658.70");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2564.23,7510.79");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4438.99,7559.10");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5216.90,7681.47");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5677.28,7601.14");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=483.89,7402.45");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1791.55,7395.91");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2146.12,7488.17");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3176.81,7450.22");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3735.64,7292.00");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4068.33,7467.14");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4946.86,7400.81");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1179.62,7169.64");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2759.65,7175.32");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4992.30,7031.38");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5606.19,7216.36");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=73.98,6855.85");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=813.38,6854.68");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1586.71,6795.42");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2043.60,6913.03");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2533.71,6904.25");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3033.97,6893.04");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3459.07,6872.55");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3898.40,6797.56");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4223.72,6945.74");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5403.87,6892.37");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5913.20,6883.85");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1164.65,6754.31");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4592.98,6755.12");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4943.86,6607.67");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5691.83,6582.80");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=156.39,6365.15");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=653.08,6507.56");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1659.46,6280.65");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2203.14,6502.10");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2787.28,6292.28");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3148.81,6396.94");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3845.54,6351.10");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4286.39,6489.38");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4672.73,6324.29");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5342.48,6442.55");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1226.16,6166.54");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2044.06,6085.06");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5020.18,6124.13");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5985.36,6228.92");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=475.70,5832.49");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=830.88,5930.58");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2635.41,5932.66");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3414.73,5996.29");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4054.35,5921.43");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4428.76,5993.64");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5630.69,5866.62");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=162.78,5591.07");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=908.54,5551.18");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1228.80,5769.89");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1704.69,5705.89");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3141.90,5674.08");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3867.84,5537.26");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5171.00,5649.90");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2534.61,5477.95");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4384.52,5366.46");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4847.58,5489.04");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5486.90,5389.84");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5885.77,5418.87");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=67.55,5243.54");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=608.04,5235.61");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1600.22,5120.75");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2117.38,5169.54");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2844.67,5246.46");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3474.37,5242.47");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4004.29,5148.00");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5764.42,5048.94");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=335.47,4922.88");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1159.66,4866.13");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2936.08,4891.31");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4694.92,4927.88");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5146.51,4810.02");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=699.02,4623.56");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1616.30,4716.21");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2185.46,4679.82");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3521.23,4754.33");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4241.88,4781.58");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=133.46,4514.17");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1293.52,4344.25");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2718.95,4399.54");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4018.74,4482.34");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4683.13,4320.43");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5060.37,4428.78");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5454.92,4407.88");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5889.33,4346.51");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=927.71,4279.48");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1989.78,4284.59");
    }

}

