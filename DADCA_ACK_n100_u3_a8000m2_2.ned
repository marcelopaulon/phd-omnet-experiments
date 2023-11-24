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
        @display("i=misc/sensor;p=404.69,7890.86");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=881.40,7865.46");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1704.12,7762.97");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3471.73,7975.24");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4676.01,7911.91");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5257.74,7990.07");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5714.64,7805.73");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1200.88,7593.82");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2405.04,7620.08");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3510.97,7577.82");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4047.42,7612.30");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4893.23,7561.02");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5300.05,7608.60");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1889.54,7335.34");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2746.57,7337.04");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3112.25,7382.41");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4493.93,7353.73");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=234.19,7156.16");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=797.07,7242.96");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1382.24,7031.25");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3698.55,7204.71");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5067.09,7246.10");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5740.64,7101.45");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=784.01,6837.37");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2717.54,6891.02");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3171.09,6970.40");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3953.41,6953.66");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4821.64,6846.25");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5414.76,6846.39");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=74.20,6727.26");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=427.44,6734.92");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1095.66,6622.13");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1709.69,6712.73");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2188.15,6645.64");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5823.88,6637.86");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1478.66,6444.20");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2653.11,6439.06");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3252.77,6509.96");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4241.28,6500.56");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4807.70,6414.72");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5208.02,6482.01");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=29.23,6263.28");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=509.96,6102.79");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1708.61,6123.11");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2173.55,6218.60");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3154.68,6137.32");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3713.85,6141.72");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4545.56,6081.28");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5490.47,6104.32");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=84.74,5910.95");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1032.28,5959.16");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1394.02,5933.55");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2776.88,5805.37");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4014.94,5895.29");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5040.58,5988.91");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5954.04,5808.86");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=80.54,5556.97");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=702.85,5712.08");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2301.26,5716.26");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5527.58,5603.79");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1187.73,5459.52");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1648.43,5465.24");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2445.53,5309.64");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3237.46,5504.18");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3688.96,5376.63");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4074.02,5364.76");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4432.16,5457.79");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5009.68,5374.25");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=639.54,5210.44");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1880.12,5169.45");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3007.38,5099.77");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5351.65,5242.68");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5966.28,5205.13");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=370.80,4911.44");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1433.54,5005.58");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2149.80,4832.01");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3455.80,4993.23");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4553.86,5006.27");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=834.44,4765.07");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1616.81,4617.14");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2493.14,4638.52");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3088.49,4669.48");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3572.93,4579.86");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4060.38,4754.72");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4389.87,4600.68");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5058.03,4630.92");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5504.76,4591.28");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=60.48,4415.62");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=485.29,4418.88");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2044.13,4426.86");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3077.74,4316.77");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3873.71,4371.35");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5755.85,4330.18");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1023.59,4279.25");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3626.55,4120.98");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4204.59,4098.99");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4790.78,4133.04");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5254.57,4168.78");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=166.39,3917.50");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=929.71,3909.03");
    }

}

