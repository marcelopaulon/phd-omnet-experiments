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

network DADCA_ACK_n100_u4_a8000m2_8
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
        @display("i=misc/sensor;p=37.67,7929.51");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1287.59,7966.53");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1803.30,7973.80");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2206.45,7980.64");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3165.62,7927.86");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3725.20,7923.39");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5917.59,7861.14");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=323.92,7663.37");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=934.71,7729.74");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1582.26,7672.40");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2680.46,7651.54");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3718.88,7564.54");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4393.00,7551.57");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4896.49,7637.85");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5394.05,7745.09");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5685.15,7523.65");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=251.86,7266.97");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=583.41,7404.77");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1141.78,7417.88");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1792.25,7275.17");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2193.62,7448.22");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3230.65,7425.12");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4687.78,7340.40");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=837.88,7018.44");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2333.47,7092.16");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2920.25,7179.89");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3470.62,7048.39");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5196.86,7116.74");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5685.82,7115.48");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=416.72,6925.16");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1767.35,6833.25");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2980.38,6833.16");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5997.19,6947.68");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1327.31,6741.09");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2391.84,6728.82");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3366.12,6691.11");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4033.02,6637.39");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4664.40,6668.01");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=234.61,6456.23");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=641.85,6316.71");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1445.25,6294.88");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2195.29,6317.34");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3039.48,6301.08");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3399.31,6292.73");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5300.33,6465.65");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5756.82,6380.84");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=399.61,6045.92");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1038.00,6130.54");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2561.09,6209.46");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4323.08,6177.15");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4866.49,6173.64");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5348.51,6033.01");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=15.52,6005.59");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1468.10,5850.96");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1901.73,5920.55");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2544.28,5827.89");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3034.55,5829.38");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3420.56,5903.14");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3993.87,5866.99");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5977.63,5781.72");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=426.30,5573.55");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=949.13,5530.76");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2184.13,5600.48");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3232.71,5540.32");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3801.41,5538.43");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4353.81,5700.12");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4732.75,5697.33");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5379.24,5623.98");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=675.94,5300.40");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1309.16,5435.36");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2906.53,5397.87");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5840.14,5387.88");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=272.06,5107.44");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1592.59,5215.60");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2511.25,5276.80");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3723.22,5145.63");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4105.42,5061.79");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4766.97,5230.72");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5296.94,5221.47");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=597.94,4846.03");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1083.34,4884.79");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2036.22,4904.32");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3211.40,4924.45");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5016.38,4807.50");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=75.63,4679.09");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1681.75,4706.66");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2798.11,4543.22");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3481.05,4536.18");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3837.06,4631.85");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4614.18,4676.08");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=500.12,4315.81");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1119.21,4518.37");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1478.02,4333.87");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2078.58,4481.44");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2472.26,4364.66");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5071.72,4344.30");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=51.92,4203.48");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1173.29,4128.89");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3197.17,4176.14");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3705.46,4202.99");
    }

}

