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

network DADCA_ACK_n100_u4_a8000m2_7
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
        @display("i=misc/sensor;p=1372.07,7756.90");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1731.01,7932.59");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4577.25,7832.77");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5143.16,7856.09");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1983.90,7658.19");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2722.41,7581.29");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5632.23,7596.36");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=270.07,7432.93");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=796.54,7316.90");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1142.97,7444.45");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2445.59,7314.81");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3359.01,7307.54");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4241.25,7497.99");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4880.30,7341.99");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1534.11,7168.63");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2029.41,7114.77");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3025.38,7124.94");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3724.39,7119.65");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4507.97,7015.05");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5231.94,7230.88");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=425.68,6998.84");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1017.74,6990.27");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1708.80,6778.77");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2583.68,6983.10");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3537.18,6805.01");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5848.93,6937.25");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=865.08,6581.50");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1264.31,6741.15");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3175.19,6622.85");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3815.45,6588.80");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4438.17,6520.50");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5007.79,6593.87");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=38.26,6480.77");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2313.16,6501.38");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2682.41,6408.13");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3449.45,6293.29");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5477.65,6407.29");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=407.14,6048.14");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1831.99,6162.45");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2358.97,6033.71");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2861.53,6029.45");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4602.48,6136.94");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5060.16,6118.84");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5998.71,6112.89");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=821.72,5956.80");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1320.35,5997.50");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3323.23,5933.90");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3939.43,6016.04");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5402.51,6002.98");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=341.39,5571.31");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1399.38,5578.83");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2660.68,5587.10");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4857.00,5639.59");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5553.58,5636.20");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5970.53,5685.56");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=752.80,5485.65");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1772.98,5465.87");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2304.56,5524.13");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3655.31,5481.01");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4260.62,5414.14");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5090.08,5292.12");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=51.70,5212.83");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=789.73,5031.26");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2635.02,5102.97");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3221.28,5201.28");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3756.40,5062.75");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4720.10,5089.51");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5638.54,5119.71");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=164.82,4813.74");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1178.64,5021.66");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1607.93,4786.24");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2265.73,5021.08");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2853.22,4789.59");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4424.72,4789.93");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5052.17,4871.09");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5956.36,4943.18");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=666.89,4638.68");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2004.41,4714.56");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3390.06,4772.47");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5429.74,4671.10");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=185.47,4316.82");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1070.99,4515.64");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1481.91,4405.97");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1825.69,4317.40");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2282.28,4462.64");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2920.32,4293.41");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3829.23,4462.19");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4650.52,4498.15");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=659.37,4163.86");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1023.88,4127.00");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3321.27,4283.69");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4258.41,4185.16");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5165.41,4221.79");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1591.60,3810.16");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2068.02,3968.83");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2444.49,4020.03");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3500.45,3838.43");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3915.70,3964.46");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4675.26,3897.85");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5544.19,3844.50");
    }

}

