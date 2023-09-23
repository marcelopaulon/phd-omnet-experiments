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

network DADCA_ACK_n100_u4_a8000m2_4
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
        @display("i=misc/sensor;p=102.79,7844.66");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=965.62,7828.05");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1569.14,7819.06");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3255.98,7764.75");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3657.17,7795.46");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4842.14,7913.40");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5325.66,7893.16");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=422.05,7547.41");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1198.93,7550.87");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1917.17,7652.96");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2379.65,7622.08");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2910.55,7699.17");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3887.38,7526.98");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4252.01,7686.75");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5715.60,7687.95");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=26.35,7336.69");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=855.32,7362.60");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1836.25,7277.17");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2691.36,7392.52");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3355.98,7391.79");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4781.26,7480.71");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5218.56,7428.78");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1315.85,7122.53");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3083.51,7153.01");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3771.41,7120.89");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4256.17,7042.99");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4747.12,7011.36");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=324.38,6864.03");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=828.21,6838.93");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2573.28,6993.22");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5655.67,6994.90");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1085.28,6555.29");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1465.28,6700.65");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2057.60,6583.77");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3244.96,6600.22");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3672.95,6546.08");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4025.77,6753.78");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4496.54,6568.49");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5207.42,6604.67");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5872.68,6673.65");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=264.37,6432.45");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=731.16,6502.46");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2357.69,6393.29");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2782.90,6406.53");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4127.93,6297.83");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5564.86,6503.11");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=550.43,6052.48");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1042.13,6143.03");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1407.00,6213.11");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1813.55,6098.83");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2596.36,6078.62");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2991.83,6104.02");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3391.53,6205.91");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5487.16,6099.18");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5977.73,6182.61");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=31.23,5844.76");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2126.18,5888.49");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3917.24,5950.39");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4429.14,5946.27");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4958.42,5973.56");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=488.83,5571.49");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1183.43,5612.46");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1821.69,5651.51");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2778.63,5742.77");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3653.59,5596.77");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4141.85,5658.44");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5601.21,5650.46");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2561.83,5432.51");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4695.80,5377.83");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5202.51,5420.39");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5600.31,5280.15");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5932.29,5468.82");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=111.51,5062.57");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4061.80,5212.76");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=669.22,4995.96");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1500.87,5007.65");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2052.39,5018.84");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2933.13,5005.50");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3258.24,4873.54");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3852.37,4890.09");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4399.97,4976.71");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4947.02,4883.41");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5616.61,4877.13");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=503.22,4586.05");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1600.42,4630.88");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5217.95,4623.15");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=187.49,4413.30");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1156.78,4530.97");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1893.08,4309.11");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2574.09,4315.80");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2931.27,4294.65");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3232.77,4474.51");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4267.05,4396.47");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4627.38,4500.33");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5939.33,4461.29");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5.93,4080.83");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=643.82,4121.58");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1320.80,4060.35");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3590.59,4105.87");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3925.74,4265.50");
    }

}

