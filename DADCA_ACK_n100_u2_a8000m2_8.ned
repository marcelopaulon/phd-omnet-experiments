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

network DADCA_ACK_n100_u2_a8000m2_8
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
        @display("i=misc/sensor;p=401.71,7937.71");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=945.34,7884.52");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1479.88,7909.34");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2136.52,7843.00");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3542.72,7844.03");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4944.47,7876.11");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5304.17,7916.32");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=546.31,7507.28");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2556.27,7590.10");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2924.54,7654.08");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4013.87,7709.95");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4672.02,7633.70");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5726.03,7690.91");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=214.29,7276.65");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1135.63,7469.61");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2836.17,7276.48");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4249.59,7437.72");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5270.05,7454.84");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5812.59,7277.05");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=798.50,7130.29");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2320.31,7206.03");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3452.81,7254.87");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3854.82,7189.06");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4454.26,7048.19");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4808.03,7211.09");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5574.43,7016.93");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=487.54,6878.80");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1305.93,6993.80");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3157.25,7008.20");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3658.01,6818.36");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5125.77,6962.09");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=127.33,6655.63");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2299.33,6691.50");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4216.05,6734.59");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4905.37,6528.45");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5793.92,6520.70");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1004.91,6485.63");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2017.56,6412.55");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2392.21,6277.56");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2707.50,6486.06");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3078.31,6507.11");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3503.86,6475.78");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4002.68,6271.26");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4552.20,6418.83");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5379.94,6448.60");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=476.49,6191.74");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1224.41,6209.62");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1761.71,6108.88");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3123.85,6087.50");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5147.46,6127.58");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=338.39,5820.97");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2270.45,5930.68");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2735.91,5997.34");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3463.26,5948.59");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4027.93,5845.30");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4760.49,5915.57");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5440.78,5880.70");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5946.71,5970.95");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=795.13,5729.17");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1422.37,5654.42");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2275.23,5529.23");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2958.15,5714.44");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3423.58,5589.92");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4354.04,5581.23");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5758.73,5625.70");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=456.67,5442.46");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1889.50,5349.29");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3142.18,5303.47");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3767.44,5433.03");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4928.47,5504.25");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=33.48,5231.79");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=610.77,5098.44");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1248.68,5234.47");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2789.53,5220.40");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4777.24,5071.78");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5278.86,5254.58");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5681.26,5257.10");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=948.00,4894.86");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1341.25,4806.27");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1718.61,4953.53");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2130.09,4908.57");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2633.10,4875.04");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3481.20,4979.19");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4088.22,5015.77");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5393.56,4917.66");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=382.94,4776.24");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=733.96,4543.06");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3089.28,4631.31");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3765.84,4631.96");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4844.53,4642.87");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5942.99,4762.75");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1341.45,4371.07");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1711.35,4515.44");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2090.66,4522.90");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2528.27,4512.71");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4412.29,4475.17");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=194.44,4247.19");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2951.48,4114.33");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3636.14,4197.57");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5101.83,4135.96");
    }

}

