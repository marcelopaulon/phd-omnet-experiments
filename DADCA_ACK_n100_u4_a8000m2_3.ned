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

network DADCA_ACK_n100_u4_a8000m2_3
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
        @display("i=misc/sensor;p=135.75,7753.78");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=637.01,7884.64");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2061.44,7777.32");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2945.25,7846.52");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3408.57,7886.59");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4030.92,7958.68");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4688.90,7783.15");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5112.29,7760.96");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5765.73,7963.19");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=982.92,7706.32");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1576.36,7631.34");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2511.44,7663.00");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3779.69,7628.12");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4169.93,7603.80");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=246.75,7391.39");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=771.03,7334.57");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1479.57,7294.85");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2792.04,7303.55");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3142.97,7315.55");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4616.89,7345.81");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5417.04,7484.26");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1091.45,7116.89");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1946.05,7192.66");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2448.08,7216.93");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3637.82,7249.63");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4884.46,7104.99");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5577.04,7115.58");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=436.77,6918.74");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1912.24,6842.88");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2532.31,6820.00");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3530.42,6859.15");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4045.30,6804.14");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4379.41,7005.56");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5941.75,6966.44");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=857.68,6737.27");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1439.68,6607.48");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2206.61,6599.49");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3055.52,6740.56");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4555.97,6682.95");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4946.34,6559.93");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5442.64,6688.37");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=518.70,6328.50");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2703.96,6317.73");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3666.46,6272.68");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4240.60,6488.20");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5439.52,6325.79");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5932.49,6510.88");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=12.55,6224.44");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1035.87,6083.92");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1539.43,6087.61");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1913.19,6182.55");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2418.90,6101.06");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3330.03,6153.06");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4825.31,6146.91");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=599.65,5806.59");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2865.88,5951.70");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3193.50,5814.54");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3629.52,5901.53");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4000.76,5964.46");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4366.33,5996.51");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5564.21,5941.38");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=251.13,5703.10");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1130.81,5669.04");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1651.28,5549.89");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2282.95,5700.47");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2767.05,5605.13");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3803.35,5553.44");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4220.36,5531.35");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4860.79,5570.08");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=674.67,5320.71");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2105.96,5381.15");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3190.06,5412.31");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5382.59,5307.71");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5735.17,5398.81");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=124.51,5043.74");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1090.15,5090.79");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1555.21,5164.26");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3021.19,5070.21");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3667.12,5131.08");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4404.69,5160.16");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=456.26,4806.05");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1343.49,4806.95");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1992.18,4892.99");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3982.49,4937.10");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4467.98,4784.57");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5412.17,4837.43");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=784.83,4664.74");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2486.91,4736.73");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2950.31,4624.98");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3453.14,4553.92");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4876.51,4742.87");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5879.05,4583.52");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=325.49,4444.69");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=920.42,4319.90");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1398.68,4412.80");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2061.97,4339.57");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2437.96,4364.45");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4021.76,4358.01");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4595.90,4446.74");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5258.36,4330.34");
    }

}

