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
        @display("i=misc/sensor;p=86.10,7884.03");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=964.54,7902.37");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2779.38,7935.97");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3775.97,7958.01");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4470.75,7962.76");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4897.34,7769.79");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5275.47,7974.53");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5892.28,7919.35");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=545.72,7748.39");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1298.69,7612.80");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1780.44,7731.01");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3596.22,7528.28");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4062.11,7521.00");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4510.17,7546.28");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5371.40,7616.71");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=46.46,7472.27");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=740.64,7419.69");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1173.46,7282.45");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1539.08,7285.36");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2076.90,7272.50");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2373.03,7479.23");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2832.04,7275.21");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3204.43,7288.21");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=376.82,7198.40");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3770.62,7040.53");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4534.22,7096.43");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4928.27,7136.34");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5872.64,7207.29");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=57.91,6844.67");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=728.87,6908.50");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2393.67,6796.71");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2793.50,6886.33");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3289.04,6813.40");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5415.01,7005.62");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=594.45,6571.57");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1037.63,6613.91");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1439.40,6720.94");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2048.81,6543.06");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3697.73,6696.51");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4299.82,6700.21");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4689.08,6755.57");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5033.55,6653.54");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5893.44,6728.71");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5.00,6415.80");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1702.56,6445.82");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2372.78,6321.24");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2807.51,6419.07");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3193.37,6332.64");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3935.02,6394.07");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4535.27,6440.13");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5725.98,6406.94");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=338.75,6148.79");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=855.11,6202.46");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1740.36,6065.83");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5110.66,6220.58");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=832.89,5787.54");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1342.72,5934.93");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2885.83,5909.81");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3337.62,5802.62");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3766.21,5800.97");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4345.34,5968.63");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4896.26,5774.57");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5383.15,5865.70");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1840.77,5692.65");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2225.55,5761.31");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4521.45,5548.15");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2.27,5428.95");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=348.34,5509.12");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2960.09,5497.47");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5277.99,5367.61");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5946.82,5473.59");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=769.71,5252.75");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3222.81,5258.80");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3748.90,5181.62");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4109.34,5120.79");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4267.73,4787.89");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4874.06,5013.36");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5798.97,4849.94");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2909.57,4744.63");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3579.55,4715.09");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3945.96,4598.79");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5321.09,4691.80");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2843.85,4382.88");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3325.76,4391.69");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4785.21,4392.69");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5839.97,4421.03");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3917.87,4044.87");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4394.51,4166.77");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3265.63,3900.85");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4659.68,3856.99");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5161.36,3933.28");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5520.88,3940.62");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5926.84,3915.95");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4033.38,3643.15");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5170.30,3577.01");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3374.63,3461.72");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3778.57,3304.26");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4541.41,3383.77");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4268.17,3117.78");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5127.60,3056.91");
    }

}

