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
        @display("i=misc/sensor;p=483.50,7989.02");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=836.29,7897.87");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1432.89,7790.83");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2233.04,7808.94");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2675.95,7964.46");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3752.31,7935.79");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4209.38,7752.66");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5112.58,7880.67");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1898.32,7656.50");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2543.04,7615.73");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3319.11,7749.34");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3761.03,7512.97");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4596.69,7697.64");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5777.25,7606.94");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=416.65,7445.02");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=902.05,7491.16");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2213.69,7356.29");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3012.56,7338.89");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5239.99,7286.54");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=484.05,7086.44");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1466.55,7186.10");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3392.99,7226.01");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4246.06,7147.09");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4653.26,7090.67");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5578.27,7017.47");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5986.13,7083.45");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=849.99,6982.81");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1582.35,6846.82");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1944.10,6963.38");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3700.69,6893.27");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1046.13,6690.08");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2585.38,6699.40");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3181.63,6702.81");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4272.47,6657.95");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4865.08,6721.31");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5316.23,6628.11");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5686.89,6646.79");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=412.99,6493.55");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1401.68,6374.40");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2012.10,6371.91");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3085.20,6347.13");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3587.01,6513.02");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=143.41,6232.12");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=724.76,6028.36");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1046.26,6168.15");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1639.91,6089.18");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2257.20,6030.48");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4130.58,6245.75");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4536.71,6187.47");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4956.07,6091.72");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1331.90,5785.07");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2732.78,6002.28");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3368.92,6017.16");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3836.05,5818.86");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4344.32,5892.71");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5152.71,5786.20");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5511.75,5965.12");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=227.56,5695.03");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1734.33,5562.87");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2580.62,5652.66");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3203.67,5528.39");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4504.51,5538.32");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5798.72,5691.49");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=546.11,5389.03");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2176.53,5387.95");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4166.78,5321.58");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4931.28,5418.82");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5851.69,5320.63");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=904.47,5254.61");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1502.91,5218.72");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2632.83,5260.27");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3894.30,5085.03");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5258.81,5276.57");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=165.16,4906.38");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1544.43,4805.12");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2038.66,4966.21");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2721.96,4836.52");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3284.11,4799.40");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4371.59,4963.37");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4741.83,4937.48");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5246.24,4833.19");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5679.64,4990.91");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=538.41,4722.09");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1198.31,4607.23");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1896.69,4605.41");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4003.32,4650.48");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4951.29,4595.17");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=292.82,4410.05");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=844.42,4521.11");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1631.01,4344.24");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2490.31,4530.07");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3321.92,4302.87");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4105.74,4315.05");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4468.81,4325.60");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5420.72,4364.25");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5884.82,4305.48");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=958.90,4131.93");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2106.43,4211.34");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2757.74,4255.78");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3735.61,4277.63");
    }

}

