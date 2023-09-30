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

network DADCA_ACK_n100_u8_a8000m2_2
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
        @display("i=misc/sensor;p=318.32,7788.87");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1142.55,7992.49");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1615.48,7793.07");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2145.52,7792.54");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3501.71,7943.59");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4448.24,7881.74");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4929.04,7909.87");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5339.20,7911.74");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=629.14,7552.71");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=981.12,7672.47");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2538.27,7690.40");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3040.75,7589.49");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3964.57,7728.54");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5858.99,7608.29");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1915.88,7308.76");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3789.59,7360.31");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4352.35,7500.12");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5329.44,7379.69");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5918.23,7259.21");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=401.14,7233.83");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1015.51,7256.55");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2572.76,7190.69");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3495.76,7120.26");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4175.62,7180.74");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4724.09,7255.65");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=109.12,6974.13");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=655.15,6882.94");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1092.01,6885.62");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1639.08,6995.00");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2243.20,6898.77");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2892.05,6983.80");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3927.62,6875.43");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4459.63,6852.75");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4909.69,6902.79");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5582.21,6993.33");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1743.71,6655.31");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2549.19,6658.47");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3233.86,6534.11");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3579.93,6752.07");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5223.79,6744.96");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=359.21,6363.40");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1008.57,6491.31");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2315.62,6348.27");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3543.69,6315.29");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4257.06,6462.20");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4749.33,6392.56");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5853.66,6510.20");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=719.31,6102.93");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1627.32,6101.61");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2858.06,6050.19");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3983.50,6100.74");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4379.95,6042.74");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4795.36,6026.75");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5604.28,6251.74");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=113.78,6001.24");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1225.93,5829.32");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1955.06,5842.07");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2375.67,5807.48");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3247.75,5995.38");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5250.12,5929.18");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5993.18,5943.08");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=172.68,5583.14");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=666.48,5741.44");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3761.05,5684.09");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4533.83,5644.98");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5263.67,5530.02");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=837.64,5371.15");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1442.28,5508.24");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1797.43,5449.23");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2225.45,5345.62");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2987.31,5364.73");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4158.79,5478.01");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4924.36,5378.49");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5800.76,5503.63");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=424.78,5246.62");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1144.42,5126.72");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1736.91,5060.28");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2646.95,5208.52");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3353.93,5217.88");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4565.81,5223.45");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=34.62,4994.34");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2332.88,4861.87");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3026.72,4960.57");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4134.99,5007.92");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4539.68,4855.18");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4955.75,4972.51");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5373.70,5011.53");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=625.31,4591.30");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1235.06,4626.18");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1895.29,4736.38");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2917.82,4570.63");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3353.98,4737.67");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3940.16,4551.21");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5168.15,4604.15");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5636.13,4696.83");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=45.33,4526.25");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2489.01,4317.63");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4314.16,4504.02");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4758.67,4334.86");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=695.76,4232.62");
    }

}

