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

network DADCA_ACK_n100_u8_a8000m2_0
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
        @display("i=misc/sensor;p=14.10,7812.76");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=482.80,7969.28");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=856.75,7777.40");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1221.56,7987.82");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3001.94,7888.03");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3682.81,7958.86");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4089.39,7927.90");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5095.84,7797.79");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5784.60,7802.91");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1471.67,7594.67");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2160.74,7693.92");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2637.84,7750.66");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3645.39,7543.68");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4714.67,7709.23");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=321.27,7369.27");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1969.96,7388.12");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2589.91,7324.30");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2951.02,7419.84");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4003.63,7293.36");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5672.35,7458.86");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=965.45,7205.25");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1315.51,7196.56");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1757.38,7068.09");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2257.83,7103.67");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3442.00,7226.31");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5022.84,7236.79");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=37.96,6797.29");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=430.41,6876.05");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1008.91,6772.90");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4527.13,6803.73");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5537.32,6957.69");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5951.42,6806.47");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1374.60,6683.09");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1912.32,6733.47");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2286.99,6744.83");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3036.63,6714.68");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3834.56,6726.70");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5153.43,6636.05");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=822.50,6306.15");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1616.81,6386.36");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2214.87,6400.86");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2619.91,6508.18");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3430.44,6356.32");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4058.94,6349.89");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4583.85,6357.75");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5593.62,6421.30");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=190.39,6227.22");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1271.20,6056.15");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2194.72,6035.22");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2678.65,6104.66");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3291.03,6032.15");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=829.83,5781.49");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1783.40,5842.32");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4295.91,5914.50");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4813.53,5894.92");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5173.08,6003.14");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5984.40,5884.10");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=193.80,5766.96");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2415.86,5653.56");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3008.96,5675.45");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3639.23,5711.41");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3976.74,5534.60");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5261.87,5633.36");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=321.96,5353.99");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=817.78,5369.85");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1369.94,5383.73");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1941.73,5444.77");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2734.04,5407.84");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3355.94,5404.65");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4322.62,5426.86");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4694.71,5515.89");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5571.01,5370.07");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=531.28,5067.56");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1098.78,5143.48");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3573.72,5031.06");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4990.78,5208.51");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=58.29,4983.56");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1438.99,4952.21");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2030.91,4981.22");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2375.20,4883.89");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2918.14,4908.78");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3930.21,4940.20");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4319.99,4991.42");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5449.35,5012.91");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3690.27,4631.69");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4530.06,4687.49");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5202.56,4636.07");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=17.13,4356.48");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=355.88,4527.91");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=820.82,4323.26");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1266.54,4343.61");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1638.81,4375.41");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2757.11,4450.43");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3162.79,4309.59");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3955.98,4378.57");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5544.22,4482.65");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2004.58,4126.08");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2337.16,4239.54");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4178.06,4050.60");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4866.95,4244.11");
    }

}

