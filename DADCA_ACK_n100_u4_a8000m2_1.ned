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

network DADCA_ACK_n100_u4_a8000m2_1
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
        @display("i=misc/sensor;p=1110.96,7966.18");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1544.94,7994.70");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2469.32,7892.56");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3128.80,7775.26");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3481.75,7981.51");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4056.07,7774.93");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4469.94,7907.58");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=367.89,7738.17");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1492.42,7638.51");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1827.87,7738.83");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2339.06,7549.78");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3613.74,7607.03");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4851.88,7722.47");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5354.17,7748.40");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=684.52,7439.46");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1107.40,7388.51");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2663.86,7378.10");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3153.11,7386.87");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4467.84,7300.85");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4984.28,7272.28");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=195.55,7118.05");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1768.95,7223.86");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3956.63,7169.37");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5365.97,7162.62");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5877.63,7161.78");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=437.17,6835.31");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=963.11,6936.79");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1437.55,6787.01");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2282.84,6955.91");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3471.51,6775.97");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4070.58,6821.64");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4418.07,6871.98");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4935.06,6870.01");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1781.06,6536.99");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2769.21,6760.43");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5353.73,6673.67");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5808.86,6653.85");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=26.35,6379.69");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=533.47,6488.88");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=823.91,6290.42");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1180.83,6368.67");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2339.95,6326.53");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3212.70,6348.94");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4589.28,6401.49");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1696.28,6187.79");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2670.12,6162.36");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4076.26,6259.41");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5068.04,6035.58");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5568.26,6138.09");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=271.63,6003.50");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1116.65,5946.08");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1679.32,5790.69");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2138.46,5866.46");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3670.72,6009.37");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4442.72,6016.20");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5969.61,5866.79");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=680.81,5745.64");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1369.37,5607.44");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2866.87,5625.33");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3950.78,5553.19");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4631.56,5568.15");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5498.84,5525.41");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=358.80,5392.43");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1156.33,5289.29");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1849.90,5425.83");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2527.43,5292.69");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3437.97,5390.46");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5055.65,5314.99");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=747.28,5162.23");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2844.46,5141.03");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3522.04,5046.43");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3849.56,5213.81");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4366.27,5038.19");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5830.76,5060.95");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=151.39,4984.89");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1972.11,4800.30");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3857.26,4834.05");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5258.38,4887.36");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=538.36,4771.41");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=899.80,4681.99");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1555.73,4598.45");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2623.41,4729.01");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3273.21,4749.04");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4920.37,4644.41");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5957.35,4715.05");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=256.03,4434.91");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2601.50,4296.57");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3134.08,4396.04");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3761.76,4495.32");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4286.89,4399.40");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5509.35,4462.70");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=603.08,4152.96");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1150.82,4059.35");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2187.74,4277.01");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3435.40,4087.99");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4640.68,4285.57");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5428.37,4048.48");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5923.70,4081.54");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=40.37,4012.19");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=401.33,3863.22");
    }

}

