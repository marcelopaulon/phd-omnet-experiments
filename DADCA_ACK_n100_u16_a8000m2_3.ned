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

network DADCA_ACK_n100_u16_a8000m2_3
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
        @display("i=misc/sensor;p=2357.56,7847.88");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2789.27,7763.77");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3393.71,7880.97");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4588.74,7986.69");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5130.86,7966.35");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5577.63,7808.92");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3743.72,7733.97");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4167.08,7658.36");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5045.99,7548.64");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5948.29,7727.30");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1725.32,7290.90");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2264.34,7464.32");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2657.82,7317.45");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4670.89,7442.82");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1263.39,7047.78");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3224.40,7228.66");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3744.52,7144.92");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4235.57,7038.56");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5529.57,7178.86");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5964.27,7101.50");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=10.46,6965.51");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=544.48,6816.53");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2107.44,6927.37");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3348.31,6893.38");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3731.79,6768.16");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4692.22,6976.91");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5146.89,6814.41");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1687.91,6576.08");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2704.58,6750.51");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4140.86,6599.74");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4697.10,6549.12");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=517.05,6357.67");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1229.10,6391.52");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2266.37,6293.71");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3100.74,6299.58");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3631.74,6410.59");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=54.64,6101.41");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1831.69,6104.83");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4185.36,6076.69");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4986.94,6238.71");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=724.57,5987.31");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1063.38,5834.65");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1482.96,5950.99");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3584.12,5998.01");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4738.21,5898.54");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5562.91,5921.76");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=220.43,5652.92");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2207.63,5623.87");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2730.39,5573.97");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3424.37,5631.41");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3829.37,5633.13");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5165.30,5667.23");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=784.24,5511.98");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1620.61,5309.69");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4385.97,5495.25");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4790.39,5320.64");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=139.55,5034.50");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=472.32,5178.90");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=955.03,5168.99");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2276.28,5053.24");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2889.45,5256.38");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3337.39,5231.14");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3809.34,5136.70");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4239.82,5121.71");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5452.11,5159.99");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5800.59,5269.88");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1579.89,4961.16");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2983.82,4804.50");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4585.05,5003.36");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4946.63,4951.67");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5919.36,4905.50");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=414.05,4715.06");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1197.24,4782.37");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1936.69,4775.05");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2562.24,4727.70");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3352.39,4774.37");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4074.50,4559.71");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4686.99,4622.49");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5104.01,4570.82");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5543.83,4706.10");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=49.48,4413.07");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=826.69,4452.06");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1476.36,4461.24");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3174.86,4413.58");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3680.21,4298.48");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4518.99,4297.75");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5487.89,4357.86");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=627.45,4122.49");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1822.92,4272.80");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2203.39,4162.28");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2754.47,4185.86");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5142.26,4222.10");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=42.13,3900.80");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1046.53,3912.42");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1387.07,4032.20");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2366.59,3816.14");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3254.03,3879.67");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3865.63,3931.71");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4634.18,3867.13");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5362.78,3807.06");
    }

}

