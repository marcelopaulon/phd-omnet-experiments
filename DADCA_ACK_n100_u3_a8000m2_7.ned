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

network DADCA_ACK_n100_u3_a8000m2_7
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
        @display("i=misc/sensor;p=322.50,7833.16");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3406.22,7896.72");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4533.47,7942.87");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4959.09,7904.51");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5805.73,7855.07");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=747.76,7525.47");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1176.63,7600.42");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1803.60,7744.58");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2150.92,7524.77");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2559.36,7752.15");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3888.70,7660.93");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4978.39,7508.00");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5406.56,7738.06");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1646.67,7274.39");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2505.01,7398.37");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3854.33,7266.03");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4259.22,7454.56");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5575.12,7328.87");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=330.37,7169.52");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1005.73,7188.08");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2227.71,7167.01");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2832.64,7239.38");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3334.78,7237.97");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4789.49,7060.72");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1938.83,6774.58");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2991.87,6842.06");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3390.32,6870.45");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3782.30,6804.10");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4388.66,6849.26");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5299.11,6856.66");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5784.76,6975.33");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=347.35,6550.09");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=829.76,6643.12");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1267.26,6697.41");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2543.38,6703.30");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4886.51,6694.29");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5736.53,6561.30");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1534.45,6342.76");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2215.35,6276.11");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2973.43,6473.40");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3616.46,6427.48");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4000.28,6447.22");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4591.55,6480.07");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5171.91,6490.23");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=286.81,6203.46");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=888.10,6107.45");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2676.10,6262.05");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4912.47,6091.29");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5438.11,6241.16");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1349.27,6017.40");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2208.04,5885.77");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2569.16,5921.54");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3135.66,5982.06");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3747.32,5843.49");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4170.79,5868.61");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4687.08,5809.01");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5586.70,5893.95");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=417.24,5552.19");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1496.15,5619.64");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2861.20,5629.08");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3964.16,5558.57");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5256.15,5589.34");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1003.39,5500.69");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2002.13,5389.81");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2394.63,5508.42");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3248.90,5361.99");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5761.51,5477.17");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=117.75,5264.78");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=720.48,5071.50");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1095.25,5089.66");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1618.28,5227.66");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2447.47,5099.43");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2937.60,5192.36");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3655.80,5039.35");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4669.96,5184.04");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5506.00,5208.64");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=351.08,4977.99");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4009.05,4927.30");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4824.45,4869.89");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5994.47,5027.99");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1094.31,4653.50");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1518.32,4610.01");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1901.23,4704.08");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2362.59,4615.43");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2919.66,4774.68");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3271.43,4588.46");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5337.90,4592.36");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5761.69,4747.30");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=99.78,4384.14");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=593.60,4407.03");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2971.63,4310.81");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3814.58,4492.53");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4591.68,4492.11");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4976.62,4317.83");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5689.50,4403.98");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1135.66,4218.97");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2216.33,4087.13");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3687.08,4044.23");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4294.11,4064.93");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=166.37,3799.95");
    }

}

