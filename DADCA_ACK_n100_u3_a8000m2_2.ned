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
        @display("i=misc/sensor;p=49.06,7821.30");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=456.58,7979.47");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1121.86,7853.36");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1545.79,7815.30");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2087.60,7818.31");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2505.62,7808.77");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3324.65,7904.93");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3815.66,7962.26");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4730.35,7853.49");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5125.83,7821.41");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5650.07,7807.18");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=631.74,7646.99");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3066.16,7635.41");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4306.74,7542.53");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=875.82,7379.84");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1246.68,7389.84");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1767.22,7351.15");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3737.12,7393.37");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4787.15,7284.80");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5171.93,7324.55");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5837.17,7411.25");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=318.21,7167.96");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2181.58,7219.31");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2959.23,7154.85");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4039.21,7141.46");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4457.37,7123.69");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1332.01,6930.72");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2480.97,6962.61");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3682.96,6930.97");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5847.25,6848.01");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=15.70,6713.17");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=714.03,6761.79");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1916.45,6728.10");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2458.14,6600.93");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3004.80,6619.71");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3536.09,6590.05");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4203.35,6636.53");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=320.95,6342.13");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1207.30,6307.02");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3941.00,6293.26");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4968.21,6342.44");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=16.24,6037.15");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=713.40,6256.43");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3022.23,6169.99");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3401.82,6144.68");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4410.04,6085.81");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4778.76,6036.29");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5233.27,6098.33");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5641.04,6185.61");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=748.92,5804.91");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1947.05,5915.03");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2446.83,5993.28");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3901.83,5882.69");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=161.35,5686.29");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1046.96,5579.28");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1595.61,5751.58");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2302.78,5591.13");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2840.50,5607.17");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3543.28,5672.91");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4238.96,5682.25");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5691.12,5614.40");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=708.02,5380.75");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1794.77,5421.55");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3226.21,5515.10");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3872.56,5516.23");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5192.77,5451.42");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1277.15,5072.91");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2260.08,5201.91");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3054.40,5108.63");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3521.88,5130.42");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3944.23,5128.00");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4755.13,5256.33");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5096.03,5114.11");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5580.57,5272.94");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1693.09,4914.26");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2458.72,4859.39");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4397.78,4792.99");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4849.33,4817.63");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=601.91,4741.38");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1341.06,4726.57");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2037.49,4751.42");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2918.59,4574.22");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3528.19,4680.07");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3960.74,4601.03");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5281.23,4708.94");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5918.83,4771.82");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=778.38,4429.87");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1162.14,4373.12");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1822.64,4426.57");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3405.41,4337.16");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4618.55,4314.35");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5037.93,4425.60");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=425.89,4083.54");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1439.28,4058.94");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1979.47,4088.02");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2401.74,4220.11");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3071.38,4212.57");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4190.16,4112.73");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5006.20,4074.97");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5350.57,4208.96");
    }

}

