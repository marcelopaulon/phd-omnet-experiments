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

network DADCA_ACK_n100_u8_a8000m2_7
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
        @display("i=misc/sensor;p=205.77,7941.54");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=613.48,7931.52");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1290.48,7926.83");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2054.83,7880.19");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3513.43,7810.60");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3963.76,7950.42");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5153.59,7836.50");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5684.25,7893.14");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=846.99,7656.37");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2131.98,7506.40");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2670.33,7514.55");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3180.18,7678.02");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4273.15,7628.85");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4723.79,7615.90");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=197.75,7417.61");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=579.08,7411.12");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1204.06,7358.43");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1547.30,7486.11");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5099.35,7415.50");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5644.95,7441.89");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=422.47,7054.46");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=881.26,7043.71");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1338.81,7024.09");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2012.32,7018.29");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2789.02,7115.01");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3330.03,7082.72");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3755.19,7248.16");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4235.92,7165.06");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4600.13,7180.25");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1652.45,6772.17");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2454.59,6835.01");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3979.22,6901.53");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4852.55,6836.74");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=771.31,6677.94");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1132.14,6723.47");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2098.73,6570.84");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3313.19,6724.82");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4165.43,6556.57");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5210.66,6664.50");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5872.83,6634.05");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=240.44,6465.49");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=770.56,6322.66");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1430.89,6488.98");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2719.71,6425.71");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3577.72,6476.87");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4583.31,6500.39");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=244.75,6091.91");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1764.55,6101.44");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2453.61,6190.89");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4823.92,6131.16");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5284.54,6151.12");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5700.38,6177.75");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1195.17,5830.99");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3235.19,5925.38");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3862.25,5994.95");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4249.94,5979.21");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=90.42,5757.95");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=632.92,5756.66");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2407.89,5681.49");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2949.57,5699.13");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4559.31,5598.63");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5525.34,5709.57");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1463.53,5384.28");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2135.02,5405.42");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3572.06,5409.08");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4031.46,5364.50");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5047.74,5521.82");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=384.28,5059.88");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=877.79,5195.92");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1808.27,5223.12");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2759.91,5220.55");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3269.89,5127.08");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4304.80,5091.98");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4677.66,5137.69");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5492.09,5137.91");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1270.48,4850.41");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2331.11,4899.74");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3120.10,4810.54");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3850.60,4856.32");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5173.78,4877.99");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1743.25,4763.95");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4772.05,4626.70");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5740.45,4755.27");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=220.64,4431.65");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=951.29,4464.04");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1358.52,4324.92");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1903.81,4437.11");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2279.87,4476.82");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2732.41,4348.70");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3567.10,4478.85");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3959.66,4516.48");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4408.77,4491.52");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5393.29,4467.47");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=557.67,4179.77");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1812.60,4072.47");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2282.03,4054.25");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3284.15,4237.05");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4730.28,4275.49");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5100.21,4119.58");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5868.47,4264.06");
    }

}

