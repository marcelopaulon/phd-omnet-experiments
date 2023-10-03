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
        @display("i=misc/sensor;p=809.18,7782.21");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1497.54,7878.09");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2184.27,7914.12");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2643.55,7913.67");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3202.38,7833.89");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4185.31,7987.86");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5076.87,7819.78");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1799.16,7532.01");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2872.77,7624.84");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3658.02,7535.86");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4527.49,7596.43");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5647.92,7678.12");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1099.63,7284.81");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2274.58,7260.31");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3208.83,7404.05");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4024.32,7299.20");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4865.42,7481.37");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5418.13,7311.59");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=597.86,7170.98");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1553.40,7016.48");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3610.61,7056.27");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4586.10,7158.67");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5808.94,7250.98");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=45.42,6822.40");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=909.30,6789.69");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2228.00,6782.51");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4020.29,6896.82");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4996.34,6928.27");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=382.42,6640.02");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1223.75,6576.78");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1893.32,6588.20");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2883.98,6734.47");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3858.10,6541.24");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4381.02,6631.08");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4882.07,6531.25");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5326.65,6674.64");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5956.15,6536.81");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=294.49,6285.56");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1515.65,6359.30");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3427.09,6481.48");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4126.73,6293.67");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5539.56,6359.71");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=904.05,6203.26");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1299.52,6062.77");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2266.62,6249.08");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3050.64,6091.54");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3694.91,6226.38");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4558.97,6164.62");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5013.18,6067.88");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5852.74,6167.95");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5.52,5775.05");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1744.59,5838.99");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2522.71,5878.77");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3957.32,5950.22");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5495.76,5947.06");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=801.53,5556.02");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1166.83,5542.11");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2053.25,5562.66");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2728.82,5566.68");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3375.83,5599.54");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4580.66,5741.12");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4933.31,5708.46");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5926.94,5591.34");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1620.05,5282.38");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2450.94,5334.74");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3960.81,5492.75");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5302.88,5361.39");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=909.76,5042.30");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3276.07,5114.75");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4264.23,5186.42");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4725.04,5213.31");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5147.67,5031.97");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=33.14,4857.45");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=552.02,5005.67");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2130.30,4936.03");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2841.50,4910.56");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3865.09,5021.22");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4369.19,4820.72");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4728.24,4831.06");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5649.04,4791.04");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=475.67,4561.51");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=983.46,4662.10");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1411.32,4577.33");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1815.20,4539.24");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2483.60,4710.26");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3539.23,4665.91");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3923.28,4600.59");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5174.95,4555.14");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2480.38,4347.76");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3106.04,4516.44");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4368.82,4302.24");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5817.12,4319.16");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=843.33,4047.48");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1233.57,4154.51");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1653.36,4133.02");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2126.73,4280.20");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2799.74,4083.75");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3453.71,4105.02");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5016.08,4142.36");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=229.66,3952.39");
    }

}

