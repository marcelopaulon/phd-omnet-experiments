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

network DADCA_ACK_n100_u3_a8000m2_3
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
        @display("i=misc/sensor;p=374.56,7842.68");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1233.65,7939.42");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1994.64,7935.81");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3830.29,7779.50");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5348.56,7999.99");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=965.10,7567.27");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1703.74,7678.40");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2071.43,7538.73");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2755.60,7718.37");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4404.23,7715.11");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4884.52,7564.95");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5373.79,7612.28");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5730.68,7590.29");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1610.55,7332.26");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2382.53,7374.47");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3249.26,7397.74");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4015.17,7478.93");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4442.71,7301.74");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=347.21,7235.85");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=698.51,7250.28");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2124.10,7098.43");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3226.37,7010.52");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5692.98,7234.83");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=396.72,6884.04");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=769.59,6852.58");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1131.13,7002.95");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2570.10,7003.14");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2874.84,6764.57");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3639.27,6926.55");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4147.64,6873.85");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4561.26,6945.86");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5008.12,6764.41");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5404.32,7005.43");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5979.08,6994.14");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1728.93,6727.41");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3589.62,6546.48");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4630.13,6598.67");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=638.81,6492.26");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=926.53,6285.68");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1469.32,6403.65");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2310.20,6439.17");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4083.48,6493.08");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4920.95,6350.44");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5579.09,6417.59");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5957.49,6510.09");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=63.51,6132.65");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1691.49,6126.83");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2789.85,6046.18");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3319.76,6175.78");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3983.20,6096.41");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5757.46,6036.57");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=666.66,5921.53");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1292.45,5792.45");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2056.82,5938.28");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3732.23,5850.80");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4432.39,5985.17");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5204.89,5848.08");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=50.08,5741.13");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2947.12,5606.57");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3442.04,5567.34");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4720.43,5695.40");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=630.68,5517.01");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1370.35,5390.50");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1993.01,5496.78");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2517.05,5461.21");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4117.35,5334.27");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4489.42,5396.01");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4887.06,5283.84");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5402.78,5364.85");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=848.04,5172.75");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3218.03,5154.36");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5874.42,5184.51");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=398.53,4871.89");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1236.94,5028.49");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2043.98,4989.70");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2491.91,4949.05");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4520.23,4816.07");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5431.75,5008.66");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=803.41,4686.86");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1580.43,4623.22");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2960.28,4705.16");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3336.70,4781.86");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3909.22,4717.25");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=467.71,4413.68");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1043.18,4364.02");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2509.97,4513.89");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4757.88,4365.69");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5106.01,4491.18");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5465.88,4448.77");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5943.67,4400.15");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=117.28,4263.69");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1469.03,4067.64");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1969.40,4164.31");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3252.74,4187.72");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3707.30,4181.59");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4527.06,4098.13");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4188.71,3860.22");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5118.82,3855.57");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5812.06,3935.42");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=175.95,3780.84");
    }

}

