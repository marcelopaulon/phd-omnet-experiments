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

network DADCA_ACK_n100_a8000m2_2
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
        @display("i=misc/sensor;p=624.88,7822.34");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3215.43,7754.92");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3838.16,7958.94");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4395.46,7941.23");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4970.23,7995.93");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=276.13,7708.21");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1261.37,7695.25");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1703.24,7601.30");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2226.09,7730.21");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2592.52,7637.42");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4098.12,7618.24");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5226.80,7683.65");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=752.43,7387.90");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1116.57,7327.16");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3543.72,7366.98");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4548.30,7447.36");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5142.24,7312.69");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1558.35,7126.98");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2339.57,7108.47");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2828.02,7224.79");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4074.71,7166.17");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5603.00,7087.33");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=101.55,6942.71");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=449.01,6770.49");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1332.05,6814.86");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4557.54,6953.31");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4935.02,6879.06");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5407.75,6774.78");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2145.46,6661.75");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2897.05,6621.03");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3251.06,6574.76");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4088.03,6576.56");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5757.70,6582.19");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=354.01,6420.26");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=826.22,6466.43");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1684.75,6282.95");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3740.96,6297.52");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4589.32,6435.19");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4934.96,6269.16");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5381.68,6395.84");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=762.88,6101.61");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1168.36,6220.71");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2313.23,6200.77");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3110.19,6172.39");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4480.22,6102.22");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5945.98,6158.45");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=97.91,5942.54");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1236.35,5826.45");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2000.06,5908.99");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2475.74,5858.43");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3741.98,5932.39");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4133.67,5992.03");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4837.92,5788.90");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5307.23,5929.33");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5662.75,5874.77");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=53.57,5563.15");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=644.61,5750.75");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2875.46,5640.93");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3351.28,5686.64");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=392.49,5321.38");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=981.14,5336.39");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1469.97,5435.29");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1823.33,5513.98");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2374.25,5452.37");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3576.39,5299.94");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4107.44,5491.21");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4765.18,5383.05");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5430.91,5313.34");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1979.54,5095.01");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2434.61,5056.83");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3220.49,5230.25");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=803.85,4810.30");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1489.38,4799.45");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2805.62,4953.54");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3990.42,4831.92");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5244.84,4908.60");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5941.94,4985.02");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=401.58,4619.20");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1828.85,4687.70");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2246.02,4744.47");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3448.69,4697.21");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4551.65,4717.69");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1212.99,4495.98");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2019.08,4292.96");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2511.62,4497.24");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2921.75,4439.57");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3816.84,4408.47");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4246.83,4511.48");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4981.36,4442.61");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5595.41,4477.35");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=320.27,4075.79");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=696.36,4119.77");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1070.59,4054.42");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1626.38,4151.09");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2731.24,4109.27");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3536.78,4182.07");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4627.12,4231.50");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5292.88,4138.80");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5910.93,4147.75");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2321.36,3961.81");
    }

}

