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

network DADCA_ACK_n100_u8_a8000m2_2
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
        @display("i=misc/sensor;p=940.84,7789.43");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1469.88,7835.74");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1939.57,7849.36");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3748.16,7890.42");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5028.21,7884.36");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5436.68,7847.77");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=276.71,7738.97");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2395.50,7733.17");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2878.55,7714.71");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4418.64,7670.02");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5932.21,7587.41");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=35.76,7474.23");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=628.84,7387.82");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1694.15,7315.71");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3222.00,7490.17");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4821.17,7444.84");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5290.10,7494.55");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=636.07,7025.28");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1083.50,7242.53");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2201.63,7243.69");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2576.53,7086.40");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3472.94,7145.45");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4092.03,7203.66");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4443.74,7120.60");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5250.05,7067.49");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5804.38,7030.51");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=52.25,6874.71");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1882.76,6906.96");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3051.89,6792.51");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3768.73,6923.53");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4436.78,6769.40");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4877.83,6836.52");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=967.12,6624.55");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2282.96,6662.46");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3527.51,6631.51");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5622.45,6572.91");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1366.15,6418.82");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1902.19,6332.47");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2684.41,6513.04");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3269.97,6290.61");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3814.14,6411.36");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4178.77,6510.63");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4749.86,6501.59");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5948.81,6330.99");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2472.79,6194.55");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2940.04,6140.34");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4975.39,6203.48");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5402.90,6022.96");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=322.81,5903.47");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=849.86,5826.87");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1228.27,5824.51");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1757.26,5937.23");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2356.11,5848.64");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3448.90,5929.40");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4094.92,5963.74");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4462.86,5867.58");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=416.86,5536.78");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1524.85,5552.84");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2829.03,5538.99");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5862.65,5706.55");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1923.28,5485.03");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3299.60,5406.01");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3983.03,5392.76");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4460.07,5464.84");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5043.30,5505.49");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5409.44,5388.44");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=263.17,5073.18");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=894.41,5211.88");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1287.16,5254.07");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2380.62,5197.37");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4851.74,5148.27");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1950.72,4804.75");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2894.20,4896.52");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3550.41,4835.20");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3925.46,4941.15");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5118.11,4861.73");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=380.99,4642.74");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1198.69,4630.21");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2322.35,4757.02");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3202.98,4615.03");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4414.07,4726.85");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4918.47,4560.08");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5691.50,4718.03");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=838.94,4336.65");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1702.38,4487.70");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2152.88,4426.15");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2809.53,4384.46");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3890.46,4505.23");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5521.89,4345.19");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5996.06,4445.56");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=280.34,4076.17");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1886.58,4045.84");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2262.68,4091.57");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3398.16,4174.96");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4444.38,4098.01");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5065.50,4175.78");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=969.98,3793.72");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1294.42,3931.02");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2627.51,3974.10");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3687.43,3809.07");
    }

}

