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

network DADCA_ACK_n100_u4_a8000m2_0
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
        @display("i=misc/sensor;p=1434.07,7893.97");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1834.22,7844.94");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2536.76,7879.01");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3024.01,7834.59");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3652.97,7876.35");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4275.68,7843.81");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4865.39,7801.62");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=130.62,7734.71");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=502.43,7673.07");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1.35,7350.75");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=391.57,7336.42");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1023.34,7378.95");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1473.92,7279.77");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2421.39,7442.13");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2858.48,7462.67");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3455.94,7480.43");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3917.87,7264.78");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4337.42,7392.42");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4796.93,7435.55");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2099.37,7193.15");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2924.48,7103.29");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=722.04,6811.62");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1254.64,6775.01");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2504.42,6912.83");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3553.11,6916.17");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4484.10,6993.82");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5008.65,6932.06");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1768.65,6755.44");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2635.72,6551.98");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3247.09,6697.55");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4555.53,6531.94");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=143.52,6442.54");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1104.16,6454.39");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1525.24,6442.88");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1957.36,6436.90");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4023.50,6352.88");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5113.31,6338.58");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=355.54,6115.09");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=809.95,6127.17");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1295.47,6041.20");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1704.70,6120.01");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2348.08,6023.91");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2761.79,6101.34");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3245.64,6080.87");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3667.60,6084.97");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4539.62,6101.29");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5713.60,6184.88");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=669.10,5776.39");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1979.92,5860.03");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5321.24,5809.71");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1200.29,5693.63");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2487.53,5596.70");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3418.54,5667.23");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4036.64,5672.23");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4440.04,5745.82");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4834.99,5714.21");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5675.08,5745.76");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=254.31,5471.16");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1032.85,5353.85");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1492.32,5278.47");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2010.37,5325.96");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2893.24,5475.12");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3746.30,5457.43");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4555.80,5402.63");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4919.29,5324.65");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5268.59,5357.72");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5843.10,5359.44");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=651.22,5166.33");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2266.11,5042.36");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2626.28,5112.55");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5648.79,5051.66");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=84.96,4986.77");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1138.09,4839.53");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1592.77,4908.95");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3235.96,5028.63");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3601.68,5005.52");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4147.64,4911.93");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4524.52,4931.24");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=152.52,4542.70");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=666.71,4775.93");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2526.84,4628.83");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4952.85,4755.95");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5858.27,4718.95");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=514.39,4369.01");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=943.68,4400.58");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1562.92,4514.85");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1957.29,4363.26");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3106.81,4415.05");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3831.00,4415.90");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4185.14,4385.01");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4752.05,4458.73");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5344.47,4469.83");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5759.91,4366.93");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=59.22,4168.33");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1182.98,4142.39");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2534.63,4252.26");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=431.74,4013.97");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=855.79,3818.75");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1785.31,3986.13");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3349.50,3858.41");
    }

}

