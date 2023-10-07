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

network DADCA_ACK_n100_u8_a8000m2_4
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
        @display("i=misc/sensor;p=3283.26,7947.29");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=4532.70,7757.07");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4892.54,7772.13");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5551.52,7827.21");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5981.31,7829.68");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1705.45,7725.80");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2061.25,7712.57");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2714.64,7552.82");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3978.17,7582.14");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1290.98,7331.49");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1924.86,7372.28");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3509.20,7439.16");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4736.56,7426.99");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5101.05,7453.14");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5751.27,7315.71");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2240.15,7218.18");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2972.07,7173.37");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3587.27,7047.53");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4519.52,7113.34");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4918.32,7092.62");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=743.63,6780.14");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1152.73,6853.29");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1689.68,6838.96");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2723.67,6889.41");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4124.26,6958.16");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5241.36,6892.56");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=361.06,6567.66");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2018.01,6689.57");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2380.56,6581.14");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3610.62,6673.54");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4488.88,6587.55");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=793.93,6338.84");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1158.25,6341.88");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1621.09,6474.52");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2971.48,6421.73");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4843.06,6503.05");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5477.44,6493.31");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5954.07,6297.26");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1384.92,6062.48");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1964.96,6081.70");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2631.01,6249.24");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2941.16,6070.90");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3310.13,6074.15");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3834.06,6252.27");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4486.73,6219.31");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5442.57,6098.21");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=414.27,5952.97");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=801.29,5849.17");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1176.87,5774.60");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2294.67,5879.35");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5092.87,5896.33");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1516.89,5647.88");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2114.54,5548.88");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3492.72,5613.48");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4200.68,5571.12");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4546.61,5760.12");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5616.31,5676.23");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=156.56,5408.90");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=809.34,5294.74");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1162.92,5317.92");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2697.55,5511.77");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3091.58,5374.98");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4976.97,5430.92");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1575.79,5255.43");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2041.23,5183.86");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2391.83,5174.13");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3749.57,5141.83");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4523.75,5213.44");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5504.26,5104.87");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5879.94,5104.80");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=194.24,5016.03");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=763.32,4842.22");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1336.21,4996.28");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1788.12,4884.73");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2869.94,5014.63");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3335.35,4868.38");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5062.64,4977.72");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2383.21,4590.06");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2797.45,4569.19");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4027.67,4676.81");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4778.43,4649.57");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5400.85,4597.25");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=159.49,4525.82");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3129.87,4408.63");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3498.45,4381.14");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5977.64,4396.67");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=910.01,4045.55");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1631.04,4240.37");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1978.34,4285.11");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2972.89,4076.56");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3996.95,4241.56");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4439.19,4277.43");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5072.58,4083.81");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=99.75,3875.80");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=548.69,3817.14");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1236.00,3881.23");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2247.69,3988.51");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2625.85,3883.92");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3337.19,3859.05");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3839.27,3866.39");
    }

}

