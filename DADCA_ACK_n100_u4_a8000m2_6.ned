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

network DADCA_ACK_n100_u4_a8000m2_6
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
        @display("i=misc/sensor;p=353.27,7755.92");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1465.31,7846.89");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1805.73,7934.26");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3160.35,7808.07");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3933.47,7921.19");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=68.54,7525.82");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1753.89,7583.35");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5430.92,7647.95");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=847.99,7481.05");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1441.52,7392.38");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2113.32,7356.61");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2733.58,7286.71");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3231.01,7391.76");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3714.38,7490.63");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4091.42,7268.09");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=379.65,7182.33");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1013.38,7073.97");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3493.38,7060.32");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5067.12,7192.79");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5543.62,7113.49");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=685.86,6915.59");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1836.38,6913.17");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2313.53,6780.50");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2898.01,6966.58");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3830.80,6877.39");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4528.30,6929.22");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4995.76,6846.50");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5941.72,6976.50");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1429.90,6684.89");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2823.90,6598.53");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5488.96,6543.24");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=622.58,6417.00");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1557.44,6292.31");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1950.38,6309.92");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3341.87,6483.87");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4452.84,6416.59");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5047.23,6482.96");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=94.94,6173.65");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=976.51,6257.32");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1265.88,6032.25");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2376.34,6024.78");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2812.17,6192.01");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3208.16,6036.20");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3936.41,6238.72");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5909.82,6197.91");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=735.40,5957.21");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3578.25,5947.35");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4364.95,6007.20");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4873.29,5903.69");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5363.99,5973.08");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5932.88,5818.15");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=210.49,5659.56");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1143.84,5647.30");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1550.38,5622.85");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1991.19,5674.86");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2720.48,5606.75");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3732.59,5628.49");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5033.51,5532.32");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5399.35,5599.46");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=798.14,5496.49");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1348.43,5336.74");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2215.81,5361.47");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3105.65,5427.82");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4438.60,5520.88");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=7.24,5150.73");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=913.65,5119.82");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1747.57,5227.36");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3430.11,5231.76");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3994.07,5156.71");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5345.84,5248.38");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5926.83,5204.26");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=414.49,4958.46");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1833.11,4859.13");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2689.72,4886.83");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3244.81,4891.13");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4713.61,4969.27");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5112.16,4837.55");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1010.16,4758.75");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1503.66,4592.57");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2338.10,4769.49");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3280.86,4542.80");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4099.80,4653.42");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=96.40,4309.01");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=565.04,4376.39");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1102.95,4299.10");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1964.33,4522.60");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2464.74,4381.35");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2935.06,4470.70");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3761.64,4477.51");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4430.94,4426.62");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3778.24,4044.56");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4689.32,4066.07");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5042.11,4122.33");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5722.73,4270.25");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=556.15,3980.83");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=936.61,3835.80");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1339.73,3901.97");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1783.64,3963.44");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2162.84,3928.14");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2759.81,3873.19");
    }

}

