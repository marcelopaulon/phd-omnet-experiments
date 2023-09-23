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

network DADCA_ACK_n100_u4_a8000m2_9
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
        @display("i=misc/sensor;p=300.29,7882.84");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1348.24,7898.46");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1926.84,7962.88");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2298.76,7875.71");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2859.09,7922.41");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3549.09,7924.73");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4272.80,7891.51");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4984.63,7912.85");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1122.81,7551.94");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1746.75,7611.93");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2465.15,7536.21");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3732.24,7616.80");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5357.28,7612.17");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5946.51,7736.10");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=183.26,7345.10");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1410.97,7280.28");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2953.78,7360.74");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3335.56,7495.54");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4753.94,7480.98");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5992.43,7360.97");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=719.01,7124.55");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2097.88,7159.80");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2703.64,7013.15");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4006.28,7046.48");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4398.07,7010.17");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5170.37,7127.89");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5718.97,7080.05");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=162.67,6804.39");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1711.40,6863.98");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3345.95,6981.73");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=739.38,6635.53");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1123.53,6663.38");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3198.83,6649.79");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3945.34,6543.09");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4525.68,6650.65");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5110.97,6674.95");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5980.54,6729.79");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1419.87,6456.42");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2309.34,6494.37");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2876.08,6468.41");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5700.30,6437.59");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=894.77,6075.60");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1383.29,6058.93");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1932.80,6233.72");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3500.87,6187.62");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4005.61,6189.07");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4417.64,6148.88");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4902.81,6069.47");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5379.23,6183.65");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=112.72,5912.85");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=555.80,5905.08");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1160.31,5782.27");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1709.88,5874.80");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2076.29,5911.41");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2627.04,5894.46");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3032.44,5855.31");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5568.57,5806.10");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5931.40,5830.96");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3536.07,5663.55");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4121.34,5647.21");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5042.92,5745.41");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=223.42,5436.72");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1343.86,5433.80");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1738.76,5499.04");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5409.71,5406.07");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=819.57,5201.63");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2224.09,5061.32");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2649.56,5276.30");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3091.08,5198.73");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3726.50,5090.73");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4557.67,5096.99");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4915.64,5152.68");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=296.54,5001.91");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=861.62,4811.35");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1399.70,5017.20");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3368.68,4834.65");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4333.68,4802.52");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5262.53,4845.33");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5640.69,4958.56");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5990.82,4933.20");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1765.21,4665.36");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2557.22,4618.43");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2973.79,4606.76");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4960.62,4605.84");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5755.16,4544.42");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1277.62,4395.18");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3286.91,4369.16");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3853.41,4438.34");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4487.78,4388.25");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=444.57,4191.41");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=958.55,4045.64");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2361.55,4163.57");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2791.71,4153.17");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4674.05,4083.21");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5140.65,4235.52");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5981.46,4082.32");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=339.17,3834.14");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1524.91,3984.85");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2250.55,3805.74");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3256.03,3800.74");
    }

}

