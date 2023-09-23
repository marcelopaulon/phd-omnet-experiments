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

network DADCA_ACK_n100_u2_a8000m2_2
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
        @display("i=misc/sensor;p=450.95,7766.35");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1120.27,7864.36");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1881.98,7985.20");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2413.37,7915.18");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3366.72,7841.35");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4132.23,7831.05");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5001.10,7978.71");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2208.57,7576.07");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2868.79,7713.34");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3728.01,7532.07");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4622.89,7657.82");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5523.68,7722.90");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=249.52,7368.67");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=778.82,7503.13");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1756.35,7368.94");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3300.09,7438.85");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4231.84,7385.71");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5196.41,7413.90");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5864.79,7396.12");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=20.57,7033.51");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=640.38,7081.49");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1323.58,7252.46");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2246.44,7172.55");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2658.41,7124.74");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3547.64,7136.64");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5531.73,7043.90");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5907.44,7032.75");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=308.77,6828.17");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1932.40,6820.73");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4511.96,6976.89");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5197.44,6841.22");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1108.59,6694.99");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1590.72,6743.68");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2262.33,6593.02");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2861.86,6526.02");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3281.44,6702.86");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3829.83,6685.25");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4448.54,6613.96");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5617.16,6615.43");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5999.42,6687.95");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=450.70,6395.36");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1212.75,6281.81");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1614.08,6361.78");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3568.90,6400.54");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5289.47,6452.73");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2294.00,6238.50");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2789.31,6162.05");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3308.38,6055.19");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3973.87,6224.30");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4573.49,6203.17");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5011.02,6194.52");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5739.78,6135.46");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=67.58,5927.26");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=565.61,5849.13");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1795.80,5901.48");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2274.19,5837.29");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5191.74,5822.50");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=115.41,5560.88");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1146.46,5662.51");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1521.66,5559.23");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2695.68,5742.46");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3443.55,5648.04");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3790.65,5754.54");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4161.20,5739.17");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4808.71,5657.57");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=715.78,5410.72");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2337.99,5463.08");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2970.70,5406.74");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3919.73,5346.70");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5617.86,5502.58");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3169.44,5096.01");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3606.36,5153.42");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4593.72,5193.62");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5214.75,5143.90");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=297.43,4810.96");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=876.99,4858.81");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1230.04,4939.39");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1961.07,5028.97");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2534.12,4925.50");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3376.05,4784.33");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4155.12,4959.85");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4837.04,4861.39");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5673.61,4838.26");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1732.14,4625.62");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2304.20,4560.50");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2820.60,4570.92");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3862.99,4740.17");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=402.31,4358.66");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=914.21,4484.46");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3951.75,4372.37");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4584.02,4514.46");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4946.00,4346.17");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5275.55,4493.55");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5591.22,4299.64");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1224.39,4227.83");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3609.11,4287.66");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5295.91,4059.27");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5937.05,4180.34");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=537.94,3864.55");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1648.01,3981.19");
    }

}

