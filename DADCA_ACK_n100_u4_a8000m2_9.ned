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

network DADCA_ACK_n100_u4_a8000m2_9
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
        @display("i=misc/sensor;p=1857.67,7820.20");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2436.06,7886.27");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3238.52,7993.50");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4691.05,7823.18");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5459.46,7879.90");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5950.12,7886.73");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=297.43,7652.25");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=752.69,7718.93");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3061.49,7661.69");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3459.89,7596.22");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4084.74,7630.69");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5113.18,7639.27");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5728.82,7601.04");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=866.09,7314.64");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1173.63,7481.78");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1704.52,7294.77");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2180.82,7348.29");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2729.78,7324.29");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3090.24,7268.03");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4254.27,7284.81");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=261.65,7222.54");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1393.48,7049.71");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3780.52,7210.54");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4607.38,7214.38");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5666.82,7171.42");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=27.44,6779.05");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=565.70,6889.10");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1012.12,6948.26");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1865.17,6880.99");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2660.16,6971.74");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4369.80,6923.28");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4932.60,6878.11");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5290.10,6872.27");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5863.26,6825.88");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=712.14,6565.52");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2275.78,6553.61");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2991.19,6704.20");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3422.76,6742.82");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3998.27,6739.66");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=340.21,6328.14");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1442.17,6466.41");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2590.57,6379.39");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3587.36,6318.67");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3992.12,6336.32");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4488.01,6454.34");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4968.00,6289.64");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5497.64,6406.46");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=573.17,6037.78");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=939.42,6160.95");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1858.25,6187.18");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3113.43,6114.78");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2271.38,5976.35");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2671.14,5777.15");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3585.14,5830.40");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3960.39,5941.17");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4527.49,5985.66");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4887.49,5906.27");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5533.98,6019.82");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1052.94,5614.54");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1452.51,5716.57");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1899.98,5646.01");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3076.76,5584.64");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4132.84,5569.06");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5103.62,5605.09");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=703.41,5391.24");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2286.30,5385.75");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2748.40,5398.26");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4578.41,5379.46");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5779.47,5347.59");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=425.66,5058.10");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1412.81,5170.63");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2029.48,5074.75");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2935.17,5071.76");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3455.63,5045.38");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3906.73,5260.48");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4928.84,5096.92");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3838.70,4812.48");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4276.58,5014.20");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5338.02,4932.83");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5967.51,5004.68");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=212.42,4715.99");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=609.52,4744.18");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1208.51,4760.13");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2537.62,4729.05");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2978.65,4592.62");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3566.33,4591.40");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4267.16,4537.90");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4694.67,4759.30");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5219.34,4574.28");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5699.07,4657.72");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=82.78,4381.72");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=483.31,4414.51");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=977.14,4491.00");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2031.33,4408.78");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1376.66,4266.35");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1746.18,4149.72");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2589.52,4098.71");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4021.77,4252.51");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4880.63,4224.65");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5335.09,4180.98");
    }

}

