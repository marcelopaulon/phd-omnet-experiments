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

network DADCA_ACK_n100_u3_a8000m2_3
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
        @display("i=misc/sensor;p=2052.07,7937.18");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2525.60,7872.31");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2913.39,7822.09");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3538.97,7773.42");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4668.18,7981.90");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5142.34,7914.06");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5726.73,7820.32");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=650.56,7710.94");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1001.58,7672.46");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1414.42,7750.71");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2246.05,7615.39");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4190.74,7641.64");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4558.88,7612.18");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=963.35,7311.90");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1838.41,7474.68");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3090.59,7426.26");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3446.86,7297.19");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3769.69,7450.54");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5034.66,7416.18");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5564.77,7375.61");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=215.19,7118.95");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=652.24,7080.87");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1570.01,7035.70");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2508.70,7106.18");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4488.34,7174.71");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5416.31,7028.05");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5970.57,7107.66");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1835.31,6778.56");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2889.91,6923.21");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3686.22,6933.49");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4107.95,6853.32");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4848.00,6788.90");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=619.36,6579.08");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1210.12,6755.17");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2237.41,6531.69");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5770.95,6633.63");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=945.65,6385.53");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1551.69,6508.13");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2906.77,6367.58");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3293.25,6462.87");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3977.44,6449.30");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4506.62,6323.65");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5225.64,6408.57");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=542.24,6162.70");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1202.62,6097.46");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1882.79,6037.13");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3309.53,6058.71");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3676.05,6105.22");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5420.47,6042.25");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1.05,5925.30");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2694.24,5903.99");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4085.03,5931.23");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4544.63,5820.70");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5061.91,5801.67");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5831.87,5893.52");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=219.37,5615.85");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2198.26,5672.87");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3621.67,5723.08");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=667.66,5454.25");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1289.88,5352.86");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1791.69,5386.66");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2500.44,5395.65");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3067.21,5393.71");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4079.45,5348.64");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5459.61,5485.04");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=183.44,5222.23");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=921.89,5070.53");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3613.09,5175.61");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4580.04,5048.93");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4910.36,5178.93");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=409.06,4845.64");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1544.25,4984.97");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2185.76,4837.84");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2742.90,4816.98");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3123.69,4939.10");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5227.36,4904.74");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5714.43,4842.38");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1418.26,4586.14");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3440.92,4681.74");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4056.76,4688.21");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4664.96,4700.10");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5960.33,4573.87");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=22.27,4391.45");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=769.62,4354.04");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2264.23,4452.25");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3202.84,4414.89");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5351.26,4452.87");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1777.85,4175.88");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2601.78,4121.00");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3550.98,4264.69");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4537.21,4187.54");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4961.38,4243.21");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5647.57,4130.91");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1483.06,3981.71");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2108.10,3847.97");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3121.15,3914.97");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3543.76,3807.16");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4175.88,4015.45");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=165.42,3791.60");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=809.65,3712.63");
    }

}

