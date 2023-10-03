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

network DADCA_ACK_n100_u16_a8000m2_1
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
        @display("i=misc/sensor;p=1061.42,7902.54");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1586.43,7917.98");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2919.68,7784.06");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3708.25,7959.02");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5135.94,7975.52");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5584.59,7873.27");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=705.18,7713.20");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1387.27,7591.51");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1943.87,7668.98");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2455.76,7581.86");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3330.56,7592.29");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4635.33,7744.04");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5200.72,7537.31");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5939.82,7696.20");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=615.37,7363.77");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=987.15,7501.78");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1833.45,7327.93");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2924.88,7395.95");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3739.68,7421.22");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4216.83,7407.69");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=317.28,7146.98");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1044.08,7015.75");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1474.60,7212.22");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3444.10,7075.79");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4526.75,7064.80");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5496.92,7161.87");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5962.04,7092.37");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2508.75,6951.14");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2932.91,6806.96");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3846.89,6896.33");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5186.74,6933.48");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5551.38,6779.42");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=330.90,6746.41");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1553.90,6602.50");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1931.91,6613.04");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4120.98,6541.19");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4746.31,6519.79");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5922.26,6706.74");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=175.74,6333.42");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=535.61,6387.19");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1156.43,6430.17");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2315.64,6330.49");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2767.59,6403.89");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3545.09,6509.52");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5220.14,6348.80");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1599.58,6121.75");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3057.52,6167.93");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3881.05,6059.68");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4339.02,6250.87");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5521.00,6147.37");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5998.56,6125.01");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=66.15,5777.99");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=604.02,5879.04");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1152.62,5882.85");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3224.41,5832.68");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4775.05,5816.75");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1889.60,5537.67");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2289.42,5658.14");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2890.61,5683.87");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3864.23,5706.85");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4373.43,5633.40");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5123.59,5676.43");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5522.77,5575.19");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5896.85,5767.17");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=693.86,5485.62");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1255.49,5384.94");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3871.76,5330.35");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5094.16,5299.03");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=187.01,5233.16");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=957.33,5125.17");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1649.76,5091.83");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2392.34,5204.57");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3009.40,5101.29");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3535.62,5229.41");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4566.22,5149.53");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5703.57,5183.32");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=356.45,4923.22");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1186.41,4852.80");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2177.34,4833.34");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4654.46,4783.59");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5393.50,4827.58");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=756.32,4732.36");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1841.89,4675.55");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2812.87,4691.50");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4031.46,4700.94");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5026.13,4679.14");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=773.67,4334.88");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1368.46,4512.69");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3183.78,4506.30");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3750.89,4408.53");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4441.62,4492.29");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5328.12,4396.22");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1315.53,4062.28");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2340.35,4162.20");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2702.89,4137.83");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3402.01,4113.89");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4022.98,4058.04");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4699.66,4179.19");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=180.40,3950.48");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1705.03,3953.36");
    }

}

