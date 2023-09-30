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

network DADCA_ACK_n100_u8_a8000m2_9
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
        @display("i=misc/sensor;p=212.75,7954.95");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=665.33,7883.28");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1358.44,7912.74");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1949.40,7763.81");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2869.71,7912.75");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3444.76,7877.40");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4044.62,7902.95");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4401.24,7991.78");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5433.86,7988.68");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3157.29,7642.14");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3873.89,7525.50");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4580.50,7674.40");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4944.36,7736.16");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5246.92,7536.98");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=333.23,7418.02");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=798.41,7465.09");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1221.09,7296.31");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2212.96,7498.29");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2660.17,7486.75");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4219.72,7469.22");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4619.42,7259.99");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5697.14,7470.39");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1632.01,7124.32");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2222.14,7061.53");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2628.22,7067.95");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3872.55,7045.38");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5626.26,7014.89");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=915.98,6840.59");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1420.35,6832.60");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3033.05,6977.95");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3508.47,6997.45");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4475.61,6933.95");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5092.74,7005.24");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=22.89,6650.57");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=455.70,6738.32");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2145.10,6569.82");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2601.42,6716.11");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3712.27,6538.03");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4382.93,6548.00");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4727.22,6633.15");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5175.71,6619.42");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5881.98,6750.36");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=627.15,6306.14");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1138.76,6477.44");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1643.62,6461.83");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2624.83,6351.08");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3094.15,6432.07");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5532.63,6504.53");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=61.66,6114.67");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1080.20,6127.47");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1975.41,6092.44");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3442.09,6062.80");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4166.08,6192.18");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4806.84,6062.42");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5302.00,6220.50");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5718.75,6086.95");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=664.47,5903.21");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1498.59,6010.78");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2272.95,5867.02");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2921.81,5978.66");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3875.90,5858.85");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4550.79,5807.70");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=296.62,5651.26");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=994.19,5669.40");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1886.13,5621.64");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3510.16,5545.59");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4968.08,5661.19");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5420.86,5697.98");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5921.10,5528.56");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1498.14,5516.93");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2569.79,5436.59");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3121.36,5505.96");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3914.85,5360.98");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4263.08,5405.25");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4682.44,5415.41");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5034.38,5286.57");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=63.58,5080.58");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=445.92,5121.28");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=776.63,5260.77");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1371.10,5170.64");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1763.33,5257.09");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2162.96,5211.37");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2858.60,5218.04");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5373.58,5193.89");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=776.17,4908.79");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1205.94,4829.24");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1851.36,4905.63");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2259.88,4789.35");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2589.80,4975.42");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3172.11,4896.94");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3586.48,4823.52");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4090.65,4843.87");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4795.08,4976.42");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5890.05,4883.46");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=108.61,4725.59");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2717.02,4535.98");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4500.15,4643.38");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5159.83,4781.75");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5596.28,4556.99");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=351.81,4386.50");
    }

}

