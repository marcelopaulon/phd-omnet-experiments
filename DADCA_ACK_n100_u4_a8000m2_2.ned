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

network DADCA_ACK_n100_u4_a8000m2_2
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
        @display("i=misc/sensor;p=244.14,7905.62");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1265.80,7831.53");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1930.69,7944.11");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2478.02,7828.89");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2855.96,7885.97");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3240.75,7996.48");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3633.67,7849.92");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4592.39,7910.09");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5890.41,7957.06");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=835.32,7651.63");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1827.73,7552.01");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2834.38,7535.72");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3381.38,7510.61");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4257.93,7585.78");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4747.79,7568.27");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5204.10,7679.54");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=230.67,7461.63");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2306.36,7352.73");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5578.46,7334.39");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5925.44,7261.66");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=170.81,7068.12");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=721.17,7193.60");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1371.80,7216.37");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1831.28,7186.76");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3119.51,7213.37");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3608.11,7206.18");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4541.53,7131.58");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4926.63,7045.72");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=926.33,6835.23");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1466.85,6836.78");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2029.53,6763.62");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2566.61,6892.47");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4009.53,6937.35");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5375.50,6923.99");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=243.45,6516.14");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3441.04,6727.17");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4618.69,6555.22");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5984.78,6589.51");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=917.07,6480.41");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1505.47,6441.44");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2286.26,6301.02");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3727.83,6491.07");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4190.69,6428.80");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5157.83,6509.12");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5574.10,6376.19");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=413.32,6186.99");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2025.72,6051.31");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2756.39,6206.31");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3269.14,6186.21");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4343.18,6092.08");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1033.20,5970.04");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1715.14,5840.40");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3751.23,5964.93");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4815.41,5943.41");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5347.96,5832.08");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5748.24,5911.84");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=740.51,5743.57");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1967.12,5549.11");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2475.65,5731.37");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3325.24,5648.94");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4079.43,5536.84");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=280.14,5516.85");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=927.42,5430.88");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1527.06,5495.12");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2974.09,5499.77");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4423.47,5373.21");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4997.74,5480.02");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5863.88,5454.27");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1025.39,5030.18");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1405.42,5083.66");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1844.40,5188.66");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2285.00,5165.83");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3355.49,5089.70");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4791.11,5188.51");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5477.04,5263.31");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=69.71,4948.19");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=482.06,4976.63");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2775.41,4933.39");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3891.23,4851.45");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4547.26,4796.65");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5203.66,4873.96");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5946.27,4873.02");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1249.68,4628.79");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1610.98,4586.03");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2902.76,4558.06");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3410.21,4640.86");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=757.44,4445.15");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1998.61,4511.69");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3707.57,4445.08");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4823.80,4521.35");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5650.98,4413.68");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1255.85,4219.18");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2603.11,4192.31");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3003.36,4190.35");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4367.82,4224.01");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5264.61,4156.79");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5923.97,4044.03");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=775.66,3931.10");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1990.74,4011.13");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3366.35,3851.06");
    }

}

