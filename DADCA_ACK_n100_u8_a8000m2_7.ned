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

network DADCA_ACK_n100_u8_a8000m2_7
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
        @display("i=misc/sensor;p=190.03,7864.62");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=617.84,7800.48");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2108.25,7920.43");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3055.13,7974.85");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3415.47,7822.12");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3788.07,7994.19");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1146.50,7607.02");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1871.94,7596.74");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2691.30,7715.13");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3252.34,7505.64");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3627.53,7519.49");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5283.77,7737.06");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=125.94,7468.08");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=497.08,7369.43");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=881.52,7326.54");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2229.82,7482.69");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2568.34,7293.87");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4014.29,7279.22");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1090.22,7037.22");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1557.38,7150.66");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2907.93,7135.67");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3465.10,7141.81");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4953.76,7196.07");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=366.16,6838.31");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=787.55,6843.92");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1408.10,6780.09");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1924.93,7004.01");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2278.52,6981.10");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2628.51,6919.30");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3193.95,6891.49");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3928.08,6796.61");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4302.68,6832.69");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4911.23,6784.72");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2116.82,6606.25");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5587.62,6685.79");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=0.76,6501.36");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=806.93,6484.16");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1343.04,6432.01");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1769.19,6411.68");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2764.67,6283.05");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3189.72,6502.31");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3762.85,6456.22");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4366.34,6368.53");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5702.93,6294.90");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=543.36,6247.45");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2287.52,6195.73");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3529.30,6167.09");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3999.07,6073.84");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5079.64,6212.55");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=131.45,5807.78");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=551.58,5818.49");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=922.17,5960.83");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1280.41,5836.36");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1882.73,5899.58");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2678.30,5853.75");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3235.35,5962.74");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4727.05,5811.57");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5152.48,5840.36");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5944.35,6009.21");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1545.76,5562.90");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2159.86,5681.80");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3982.64,5696.25");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4387.01,5701.66");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=287.54,5347.07");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=865.92,5328.51");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2499.10,5330.34");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2994.00,5495.82");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3650.93,5523.90");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4646.43,5322.40");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5170.71,5315.70");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=527.90,5073.53");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1232.44,5148.78");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2094.01,5232.82");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2859.66,5061.11");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3295.43,5196.26");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3814.20,5178.77");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4183.43,5109.50");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=837.71,4873.38");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1603.69,5025.50");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2446.52,4901.14");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4904.50,4841.90");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=385.44,4707.58");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1948.43,4600.71");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3274.44,4775.51");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3718.30,4697.24");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4361.69,4603.33");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=462.22,4303.97");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=867.55,4318.12");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1282.74,4484.16");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2502.89,4503.45");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3552.26,4354.14");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4092.64,4306.16");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5323.52,4330.18");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=147.65,4082.72");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1383.48,4138.93");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1810.29,4227.76");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2189.49,4190.15");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4804.67,4115.36");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=506.90,3863.29");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1041.38,3846.80");
    }

}

